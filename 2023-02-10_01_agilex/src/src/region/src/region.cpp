#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose2D.h"   
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"


#define V_Region_NO  6

std_msgs::Bool getRange, getstop;
std_msgs::Int16 region_msg;

double pos_x = 0.0;
double pos_y = 0.0;
double roll,pitch,yaw;
double amcl_x = 0.0, amcl_y = 0.0;

struct Point 
{ 
	double x; 
	double y; 
	double z;
};

struct Rect_Region
{
	double top;
	double bottom;
	double left;
	double right;	
};

geometry_msgs::Pose2D my_pose;

struct Rect_Region Vision_Region[V_Region_NO];
///////////amclCallback////////////
void amclCallback(const geometry_msgs::PoseWithCovarianceStamped &amcl_data)
{
	amcl_x = amcl_data.pose.pose.position.x;
	amcl_y = amcl_data.pose.pose.position.y;
	
	printf("amcl_X = %.3lf , amcl_Y = %.3lf  \n",amcl_x,amcl_y);
}


void poseCallback(const geometry_msgs::PoseStamped& msg)
{
	my_pose.x = (double)msg.pose.position.x;
	my_pose.y = (double)msg.pose.position.y;
	
	tf2::Quaternion q(
        msg.pose.orientation.x,        msg.pose.orientation.y,
        msg.pose.orientation.z,        msg.pose.orientation.w);
        tf2::Matrix3x3 m(q);     
 
    m.getRPY(roll, pitch, yaw);
    my_pose.theta = yaw;		
}

void init_vision_region(void)            //vision control section setting
{
	Vision_Region[1].top    =  6.0;
	Vision_Region[1].bottom =  4.5;
	Vision_Region[1].left   =  2.0; 
	Vision_Region[1].right  =  -1.0;
	
	Vision_Region[2].top    =  3.75;
	Vision_Region[2].bottom =  1.0;
	Vision_Region[2].left   =  4.8; 
	Vision_Region[2].right  =  3.3;
	
	Vision_Region[3].top    =  -4.5;
	Vision_Region[3].bottom =  -6.5;
	Vision_Region[3].left   =  2.5; 
	Vision_Region[3].right  =  0.5;
	
	Vision_Region[4].top    =  -3.75;//3.83
	Vision_Region[4].bottom =  -6.5;
	Vision_Region[4].left   =  -0.2;//0.4 
	Vision_Region[4].right  =  -3.0;
	
	Vision_Region[5].top    =  -0.5;//3.83
	Vision_Region[5].bottom =  -1.8;
	Vision_Region[5].left   =  1.5;
	Vision_Region[5].right  =   0.5;//0.8
	
	
	
}


int check_car_vision_region(void)                 //비젼 제어 영역에 있는지 여부 
{
	int i, id_region = 0;
			
	for(i=0;i<V_Region_NO;i++)
	{	
  	   if(Vision_Region[i].left > Vision_Region[i].right)
	   { 
		double temp;
		temp = Vision_Region[i].left;
        Vision_Region[i].left  = Vision_Region[i].right;
        Vision_Region[i].right = temp;
	    }
	    //printf("%d : %6.3lf %6.3lf %6.3lf %6.3lf\n", i, Vision_Region[i].left, Vision_Region[i].right, Vision_Region[i].bottom, Vision_Region[i].top);
	    if(   (amcl_y>=Vision_Region[i].left) && (amcl_y<=Vision_Region[i].right) 
	       && (amcl_x>=Vision_Region[i].bottom) && (amcl_x<=Vision_Region[i].top)  )
	    {			
	      id_region = i;	      
	      break;
	    }
	}	
	return id_region;	
}



int main(int argc, char **argv)
{
  char buf[2];
  ros::init(argc, argv, "race_waypoints_navigation");

  ros::NodeHandle n;
 
  ros::Subscriber sub3 = n.subscribe("/slam_out_pose",100, &poseCallback);
  
  ros::Publisher markers_pub = n.advertise<visualization_msgs::MarkerArray>("marker/node", 1);
  ros::Publisher target_pos_pub = n.advertise<geometry_msgs::Pose2D>("/pose_goal", 10);
  ros::Publisher bool_pub = n.advertise<std_msgs::Bool>("getRange", 1);
  ////
  std::string region_topic = "/region_topic";
  ros::Publisher region_pub = n.advertise<std_msgs::Int16>(region_topic, 10);
  ///
  std::string amcl_pose_topic = "/amcl_pose";
  ros::Subscriber amcl_pose_sub      = n.subscribe(amcl_pose_topic, 10, amclCallback);

 
  ros::Rate loop_rate(10);  // 10
  
  int count = 0;
  int stop_count=0;
  double pos_error_x = 0.0;
  double pos_error_y = 0.0; 
  
  geometry_msgs::Pose2D pose_goal;
  
  Point p; std::vector<Point> vec_point; 
  visualization_msgs::MarkerArray marker_arr; 
  for (size_t i = 0; i < vec_point.size(); i++)
  { 
    Point o_marker = vec_point[i]; 
    visualization_msgs::Marker marker; 
    marker.header.frame_id = "/map"; // map frame ±âÁØ 
    marker.header.stamp = ros::Time::now(); 
    marker.type = visualization_msgs::Marker::SPHERE; 
    marker.id = i; 
    marker.action = visualization_msgs::Marker::ADD; 
    marker.pose.orientation.w = 1.0; 
    marker.pose.position.x = o_marker.x; // marker x position
    marker.pose.position.y = o_marker.y; // marker y position  
    // Points are green 
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0; 
    marker.scale.x = 0.05; 
    marker.scale.y = 0.05; 
    marker_arr.markers.push_back(marker);
  } 
  int stop_line_detection = 0;
  int vision_id = 0;
  int waypoint_id = -1;
  int wp_go_id = 0;
  init_vision_region();
  target_pos_pub.publish(pose_goal);
  
  while (ros::ok())
  {	

//	ROS_INFO(" X : %6.3lf   Y : %6.3lf ", amcl_x, amcl_y ); 
//	ROS_INFO("amcl_X = %.3lf , amcl_Y = %.3lf  \n",amcl_x,amcl_y);
	
	 //비젼 제어 영역에 있는지 여부 
	vision_id = check_car_vision_region();
	if(vision_id != -1)
	{  	
		//ROS_INFO("Vision Region : %2d",vision_id); 
	    getRange.data = true; 	    
    }
   // ROS_INFO("Vision Region : %2d",vision_id);
    
    region_msg.data = vision_id;
     
	// publish topics	
	bool_pub.publish(getRange);
	markers_pub.publish(marker_arr);
	////
	region_pub.publish(region_msg);
	 
	loop_rate.sleep();
    ros::spinOnce();
    ++count;
  }
  return 0;
}

/*
		pos_error_x = abs(my_pose.x - my_waypoints_list[0].x);
	    pos_error_y = abs(my_pose.y - my_waypoints_list[0].y); 
	    pose_goal.x = my_waypoints_list[0].x;
	    pose_goal.y = my_waypoints_list[0].y;
	    pose_goal.theta = DEG2RAD(0);
	    target_pos_pub.publish(pose_goal);
	 
	    ROS_INFO(" X : %6.3lf   Y : %6.3lf  Yaw : %6.3lf E_x : %6.3lf  E_y : %6.3lf", my_pose.x, my_pose.y, RAD2DEG(my_pose.theta), pos_error_x, pos_error_y);  
	    if(( pos_error_x<= WayPoint_X_Tor)&&( pos_error_y <= WayPoint_Y_Tor))
        {  
	       ROS_INFO("WayPoint-1"); 
	       mission_flag[0] = 1; 
	    } 
	   */

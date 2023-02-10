//mux
#include "ros/ros.h" 
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Range.h"
#include <chrono>
///////////////////////IMU//////////////////////
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "sensor_msgs/Imu.h"
////////////odom//////////////////
#include "nav_msgs/Odometry.h"

#define MAX_angluar_velocity 1
#define DEG2RAD(x) (M_PI/180.0)*x
#define RAD2DEG(x) (180.0/M_PI)*x

int lidar_flag = 0, mission = 0, sum = 0;
int crosswalk_detected = 0, crosswalk_distance = 0;
int mission_flag = 0, odom_flag =0,second_odom_flag = 0,thirth_odom_flag,odom_init_flag =0;
int Init_yaw_flag = 0;

double linear_x = 0.0, angular_z = 0.0;
double roll,pitch,yaw;
double roll_d,pitch_d,yaw_d, yaw_d_360;
double yaw_d_old  = 0.;
double target_yaw = 0.;
double init_yaw = 0.;
double steer_vel = 24;
double steer_vel_R = 26;
double Kp_y   = 0.01;
double Kd_y   = 0.04;
double Ki_y   = 0.0;

double error_y     = 0.0;
double error_d   = 0.0;
double error_sum = 0.0;
//////////////odom///////////////
double position_x = 0.0, position_y = 0.0;
double init_position_x = 0.0, init_position_y = 0.0;

///////////////////////////////////////////////
int lidar_detected = 0;
int region_no;
///////////////////////////////////////////////
float no_blob = 0;

geometry_msgs::Twist cmd_vel_msg;
std_msgs::Int16 mux_mission_msg;

int map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void lane_Callback(const geometry_msgs::Twist &cmd_input)
{
   linear_x  = cmd_input.linear.x ;//m/s
   angular_z = cmd_input.angular.z ;//rad/s
}

void crosswalk_detect_Callback(const std_msgs::Int32 &msg)
{
   if (msg.data == -1){
		crosswalk_detected = 0;
		crosswalk_distance = msg.data;
	}
	else{
		
		crosswalk_detected = 1;
		crosswalk_distance = msg.data;
	}
}

void lidar_check_Callback(const std_msgs::Int16 &msg)
{
   if (msg.data == 1){
		lidar_detected = 1;
	}
	else{
		lidar_detected = 0;
	}
}

//////////
void region_sub_Callback(const std_msgs::Int16 &msg)
{
   region_no = msg.data;
}

/////////
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) 
{ 
    tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
        tf2::Matrix3x3 m(q);
      
    m.getRPY(roll, pitch, yaw);

    yaw_d = RAD2DEG(yaw);
    
    yaw_d_360 = (yaw_d_360 < 0) ? yaw_d + 360 : yaw_d;
     
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  if(odom_init_flag == 1){
	  init_position_x = position_x;
	  init_position_y = position_y;
	  odom_init_flag = 2;
  }
  
  position_x = msg->pose.pose.position.x;
  position_y = msg->pose.pose.position.y;
 
  
}

/////////////////lidar////////////////////
void blob_Callback(const std_msgs::Float32 & msg)
{
	no_blob = msg.data;
	//printf("blob data : %d\n", no_blob);
}

double control_yaw(void)
{
	double cmd_vel_angluar_z;
	double yaw_d1;

	int error_d = 0;
	double error1;
	double CW_flag = 0.0;
	int     quotient  = 0  ;
	double  remainder = 0.0;

	double target_yaw = 76.0 ;

	quotient  = target_yaw/360.0;
	target_yaw = target_yaw - 360.0*quotient;
	
	if(target_yaw  <-180)
	{
		target_yaw += 360;		
	}

	CW_flag= sin(DEG2RAD(target_yaw-yaw_d));	
   
    //printf("sin(%6.3lf - %6.3lf) = %6.3lf\n ", target_yaw, yaw_d, CW_flag );

	error1 = target_yaw - yaw_d;	
           
    if( (target_yaw > 179) || (target_yaw < -179))
    {
		error1 = target_yaw - yaw_d_360;	
	} 		
    if(error1 > 180)
    {
		error1 -=360;
    }
    error_y = error1;
  
	cmd_vel_angluar_z = Kp_y * error_y + Kd_y * error_d + Ki_y * error_sum;
	
	error_d = error_y;
	yaw_d_old = yaw_d1;
	
	cmd_vel_angluar_z  =  (cmd_vel_angluar_z >=  MAX_angluar_velocity)?  MAX_angluar_velocity : cmd_vel_angluar_z;
	cmd_vel_angluar_z  =  (cmd_vel_angluar_z <= -MAX_angluar_velocity)? -MAX_angluar_velocity : cmd_vel_angluar_z;
	
	ROS_INFO("Yaw Angle : %6.3lf %6.3lf Error : %6.3lf | cmd_vel_angluar_z %6.3lf",yaw_d,yaw,error_y,cmd_vel_angluar_z );
	ROS_INFO("Target Yaw : %6.3lf Target_Yaw(abs) : %6.3lf ",target_yaw, fabs(target_yaw));
	return cmd_vel_angluar_z;
}


int main(int argc, char **argv)
{
	double pid_control = 0.0;
	ros::init(argc, argv, "MUX");
	ros::NodeHandle n;
	
	std::string lidar_obs_flag_topic = "obs_flag";
	std::string cmd_vel_lane_topic = "/cmd_vel_lane";
	std::string cmd_vel_output_topic = "/cmd_vel";
	std::string sub_crosswalk_detect = "/limo/crosswalk_y";
	std::string imu_topic = "/imu";
	std::string odom_topic = "/odom"; 
	std::string blob_topic = "/obs_flag" ;
	std::string lidar_check_topic = "/obs_flag_check";
	std::string mux_mission_output_topic = "/mux_mission_flag";
	std::string region_sub_topic = "/region_topic";
 
	
	ros::Subscriber lane_cmd_sub = n.subscribe(cmd_vel_lane_topic, 10, lane_Callback);
	ros::Subscriber crosswalk_detect_sub = n.subscribe(sub_crosswalk_detect, 10, crosswalk_detect_Callback);
	ros::Subscriber subIMU = n.subscribe(imu_topic,10, imuCallback);
	ros::Subscriber sub_odom = n.subscribe(odom_topic, 10, odomCallback);
	ros::Subscriber obs_check_sub = n.subscribe(lidar_check_topic, 10, lidar_check_Callback);
	ros::Subscriber region_sub = n.subscribe(region_sub_topic, 10, region_sub_Callback);
	//////////
	ros::Subscriber sub_blob_no = n.subscribe(blob_topic, 10, blob_Callback);
	
	ros::Publisher  cmd_vel_pub =  n.advertise<geometry_msgs::Twist>(cmd_vel_output_topic, 10);
	ros::Publisher  mux_mission_pub =  n.advertise<std_msgs::Int16>(mux_mission_output_topic, 10);
	
	ros::Rate loop_rate(10.0); //10.0HZ
	 
	while(ros::ok())
  {
	  pid_control = control_yaw();
	  switch(mission) {
		  case 0:{
			cmd_vel_msg.linear.x = linear_x;
			cmd_vel_msg.angular.z = angular_z;
			odom_flag = 0;
			//printf("Lane Driving!!! \n");
			///////// sorry
			//crosswalk_detected == 1;
			//mission_flag == 3;
			///////// sorry
			//if(lidar_flag == 1) mission = 1;
			if(crosswalk_detected == 1 && mission_flag == 0) 
			{
				cmd_vel_msg.linear.x = 0.0;
				cmd_vel_msg.angular.z = 0.0;
				printf("Crosswalk Detected, Stop! \n");
				ros::Duration(5.0).sleep();
				mission = 2;
			}
			if(lidar_detected == 1 && mission_flag == 1){
				//printf("Let'GO");
				odom_init_flag = 1;
				cmd_vel_msg.linear.x = 0;
		    	cmd_vel_msg.angular.z = 0;
		    	ros::Duration(3.0).sleep();
				mission = 3;
			}
			if(lidar_detected == 1 && mission_flag == 2){
				//printf("Let'GO");
				mission = 4;
				cmd_vel_msg.linear.x = 0;
		    	cmd_vel_msg.angular.z = 0;
		    	ros::Duration(3.0).sleep();
				
			}
			if(lidar_detected == 1 && mission_flag == 3){
				//printf("Let'GO");
				mission = 5;
				cmd_vel_msg.linear.x = 0;
		    	cmd_vel_msg.angular.z = 0;
		    	ros::Duration(3.0).sleep();
		     }
		     else if(lidar_detected == 0 && mission_flag == 4){
				 mission = 0;	 
		     }
			
			sum = 0;
		break;
		}
		
		  
		  case 1:{
			cmd_vel_msg.linear.x = 0.0;
			cmd_vel_msg.angular.z = 0.0;
			//printf("Obstacle_detected!!! \n");
			if(lidar_flag == 0) mission = 0;
			break;
		}
		  
		  case 2:{
			cmd_vel_msg.linear.x = 0.3;
			cmd_vel_msg.angular.z = pid_control;
			sum++;
			if(sum > 30){
				mission_flag = 1;
				mission = 0;
			}
			break;
		}
			
		  case 3:
		  {  // printf("333Let'GO3333");
			  if(odom_flag == 0)
			  {
				  if((position_x-init_position_x) > 0.33){ // 0.6 , 0.35
					cmd_vel_msg.linear.x = 0.0;
					cmd_vel_msg.angular.z = 0.0;
					odom_flag = 1;
				 }
				  else{  
					cmd_vel_msg.linear.x = 0.2;
					cmd_vel_msg.angular.z = DEG2RAD(steer_vel) ;
					
				 }
			   }
			   else if(odom_flag == 1){
				   if((position_x-init_position_x) > 0.84){ // 0.6 , 0.35
					cmd_vel_msg.linear.x = 0.0;
					cmd_vel_msg.angular.z = 0.0;
					odom_flag = 2;
				 }
				  else{  
					cmd_vel_msg.linear.x = 0.2;
					cmd_vel_msg.angular.z = pid_control;
				 }
			   }
			   else if(odom_flag == 2){
				   if((position_x-init_position_x) > 1.45){ // 0.6 , 0.35
					cmd_vel_msg.linear.x = 0.0;
					cmd_vel_msg.angular.z = 0.0;
					odom_flag = 3;
				 }
				  else{  
					cmd_vel_msg.linear.x = 0.2;
					cmd_vel_msg.angular.z = -DEG2RAD(steer_vel_R); // 
			
				 }
			   }
			   else if(odom_flag == 3){
				   mission = 0;
				   mission_flag = 2;
				   
			   }
				 
				  
		   break;	
		  } 
		  case 4:
		  {
			if(second_odom_flag == 0)
			  {   
				  cmd_vel_msg.linear.x = 0.0;
		          cmd_vel_msg.angular.z = 0.0;
		          ros::Duration(3.0).sleep();
		          second_odom_flag = 1;
			 }
			 else if(second_odom_flag == 1)
			 {
				 mission = 0;
				 mission_flag = 3;
			 }
	      break;	  
		 }
		 
		 case 5:
		  {
			if(thirth_odom_flag == 0)
			  {   
				  cmd_vel_msg.linear.x = 0.2;
		          cmd_vel_msg.angular.z = no_blob;
		       }
			 
	      break;	  
		 }
			   	 	
		  }
			
	  
	mux_mission_msg.data = mission;		
	ROS_INFO("mission = %d \n",mission);
	ROS_INFO("mission_flag = %d \n",mission_flag);
//	ROS_INFO("odom_flag = %d \n",odom_flag);
	ROS_INFO("lidar_detected = %d \n",lidar_detected);
	ROS_INFO( "\n region_no = %d  \n",region_no);

	//printf("lidar_detected = %d \n",lidar_detected);
	//ROS_INFO("LIDAR = %0.4f\n",no_blob);
	cmd_vel_pub.publish(cmd_vel_msg);
	mux_mission_pub.publish(mux_mission_msg);
	//ROS_INFO("Position-> x: [%f], y: [%f],\n", position_x- init_position_x,position_y -init_position_y );

    ros::spinOnce();
    loop_rate.sleep();
  
}
  return 0;
	
}







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


#define V_Region_NO  2

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
	Vision_Region[0].top    =  2.35;
	Vision_Region[0].bottom =  0.3;
	Vision_Region[0].left   =  0.85; 
	Vision_Region[0].right  =  -0.25;
	
	Vision_Region[1].top    =  5.88;
	Vision_Region[1].bottom =  3.5;
	Vision_Region[1].left   =  0.85; 
	Vision_Region[1].right  =  -0.25;
}


int check_car_vision_region(void)                 //비젼 제어 영역에 있는지 여부 
{
	int i, id_region = -1;
			
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
  int vision_id = -1;
  int waypoint_id = -1;
  int wp_go_id = 0;
  init_vision_region();
  target_pos_pub.publish(pose_goal);
  
  while (ros::ok())
  {	

	ROS_INFO(" X : %6.3lf   Y : %6.3lf ", amcl_x, amcl_y ); 
	
	 //비젼 제어 영역에 있는지 여부 
	vision_id = check_car_vision_region();
	if(vision_id != -1)
	{  	
		ROS_INFO("Vision Region : %2d",vision_id); 
	    getRange.data = true; 	    
    }
    
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

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


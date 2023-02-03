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

int lidar_flag = 0, mission = 0;
int crosswalk_detected = 0, crosswalk_distance = 0;
double linear_x = 0.0, angular_z = 0.0;
int mission_flag = 0;

///////////////////////////////////////////////
int cross_flag = 0;
///////////////////////////////////////////////

geometry_msgs::Twist cmd_vel_msg;

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
		cross_flag = 1;
		crosswalk_detected = 1;
		crosswalk_distance = msg.data;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "MUX");
	ros::NodeHandle n;
	
	std::string cmd_vel_lane_topic = "/cmd_vel_lane";
	std::string cmd_vel_output_topic = "/cmd_vel";
	std::string sub_crosswalk_detect = "/limo/crosswalk_y";
	
	ros::Subscriber lane_cmd_sub = n.subscribe(cmd_vel_lane_topic, 1, lane_Callback);
	ros::Subscriber crosswalk_detect_sub = n.subscribe(sub_crosswalk_detect, 1, crosswalk_detect_Callback);
	
	ros::Publisher  cmd_vel_pub =  n.advertise<geometry_msgs::Twist>(cmd_vel_output_topic, 10);
	ros::Rate loop_rate(10.0); //10.0HZ
	
	while(ros::ok())
  {
	  switch(mission_flag) {
		  
		  case 0: {
				
				cmd_vel_msg.linear.x = linear_x;
				cmd_vel_msg.angular.z = angular_z;
				printf("Lane Driving!!! \n");
				mission_flag = mission_flag + 1;
				
			  
			  }
		  case 1: {
			  
				cmd_vel_msg.linear.x = 0.0;
				cmd_vel_msg.angular.z = -0.5;
				cmd_vel_pub.publish(cmd_vel_msg);
				printf("Crosswalk Detected, Stop! \n");
				ROS_INFO("cmd_vel1 = %0.4f \n",cmd_vel_msg.linear.x);
				ros::Duration(5.0).sleep();
				printf("go go go go \n");
				
			  
			  
			  
			  }
	      default:
	      
			  
		  
	  }
	  

			if(crosswalk_detected == 1 && mission_flag == 0){
				cmd_vel_msg.linear.x = 0.0;
				cmd_vel_msg.angular.z = -0.5;
				cmd_vel_pub.publish(cmd_vel_msg);
				printf("Crosswalk Detected, Stop! \n");
				ROS_INFO("cmd_vel1 = %0.4f \n",cmd_vel_msg.linear.x);
				ros::Duration(5.0).sleep();
				printf("go go go go \n");
				/*cmd_vel_msg.linear.x = 0.3;
				cmd_vel_msg.angular.z = -0.5;
				cmd_vel_pub.publish(cmd_vel_msg);
				ros::Duration(5.0,0).sleep();*/
				mission_flag =+1;
				ROS_INFO("cmd_vel2 = %0.4f \n",cmd_vel_msg.linear.x);
			}
			if(crosswalk_detected == 0){
				
				
				cmd_vel_msg.linear.x = linear_x;
				cmd_vel_msg.angular.z = angular_z;
				printf("Lane Driving!!! \n");
				ROS_INFO("cmd_vel3 = %0.4f \n",cmd_vel_msg.linear.x);
				crosswalk_detected = 0;
				cross_flag = 0;
			}
	
			
	ROS_INFO("result cmd_vel = %0.4f \n",cmd_vel_msg.linear.x);		
	cmd_vel_pub.publish(cmd_vel_msg);
	
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
	
}

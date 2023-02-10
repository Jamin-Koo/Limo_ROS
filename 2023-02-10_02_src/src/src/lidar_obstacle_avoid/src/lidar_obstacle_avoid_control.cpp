// AA_obstacle_stop.cpp : 이 파일에는 'main' 함수가 포함됩니다. 거기서 프로그램 실행이 시작되고 종료됩니다.
//
#define DEBUG 1

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Range.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "math.h"

#include <string.h>
#include "obstacle_processing.h"

#include <iostream>

#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180.*M_PI)

#define MAX_Obstacle_dist 10 //[m]
#define LIDAR_Obstacle   0.28
#define LIDAR_Obstacle_angle 10
#define LIDAR_TUNNEL	0.8
#define LIDAR_PASS		0.9
#define LIDAR_PARKING	1.2
#define PASS_DISTANCE 	25
#define TUNNEL_DISTANCE 60
#define PARKING_DISTANCE 40
#define PARKING_DISTANCE2 65
#define OBSTACLE_DISTANCE 15

std_msgs::Int8 obs_data;
std_msgs::Int8 obs_4_data;
std_msgs::Int16 front_flag_data;
std_msgs::Int16 front_flag_data2;
std_msgs::Int16 front_right_flag_data;
std_msgs::Int16 right_flag_data;
std_msgs::Float32 tunnel_angle;
std_msgs::Int16 pass_data;

int mission_flag = 0;
int front_flag = 0;
int count = 0, distance = 0, distance2 = 0;
float lidar_steer = 0.0;

float lidar_obstacle_D = 0.0;
float center = 0.0, radius = 0.0;
float average_right;
float average_left;

double error = 0.0;
double error_delta = 0.0;
double error_old = 0.0;

double Kp = 1.5;
double Kd = 3.0;

void mission_Callback(const std_msgs::Int8 & msg)
{
	mission_flag = msg.data;
}

int map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	int count = (int)(360. / RAD2DEG(scan->angle_increment));
	double *obstacle;
	int sum = 0;
	double front_data = lidar_obstacle_D*100;
	double front_data2 = lidar_obstacle_D*100;
	double front_right_data = lidar_obstacle_D*100;
	double right_data = lidar_obstacle_D*100;
	double front2_data_old = 0, front_data_old = 0;
	int cnt_right=0;
    int cnt_left =0;
    float r_dis_sum = 0.0;
    float l_dis_sum = 0.0;
    float lidar_steer2 = 0.0;
	obstacle = new double[181];
	
	for (int i = 0; i < 181; i++) obstacle[i] = MAX_Obstacle_dist;
	for (int i = 0; i < count; i++)
	{
		int degree = (int)RAD2DEG(scan->angle_min + scan->angle_increment * i);
		if (scan->ranges[i] <= lidar_obstacle_D)
		{
		    if (((degree >= 0) && (degree < LIDAR_Obstacle_angle)) || ((degree >= 360 -LIDAR_Obstacle_angle) && (degree < 360)))
		    {
				front_data = scan->ranges[i]*100;
				(front_data == 0) ? front_data = front_data_old : front_data_old = front_data;
			//	ROS_INFO("front_data ; %0.4f \n",front_data);
				sum++;
		    }
		    if((degree<=-2.5) && (degree>=-7.5))
		    {
				front_data2 = scan->ranges[i]*100;
				(front_data2 == 0) ? front_data2 = front2_data_old : front2_data_old = front_data2;
				//if(front_data2 == 0) front_data2 = lidar_obstacle_D*100;
			}
		    if((degree<=-15) && (degree>=-25))
		    {
			  if(scan->ranges[i] >= 0.1 ) 
			  {
				front_right_data = scan->ranges[i]*100;
				if(front_right_data == 0) front_right_data = lidar_obstacle_D*100;
			  }
			}
		    /*if((degree<=-80) && (degree>=-90))
			{
			  if(scan->ranges[i] >= 0.1 ) 
			  {
				cnt_right++;
				//printf("cnt_right = %d\n", cnt_right);	
				r_dis_sum += scan->ranges[i]*100;	
				average_right = -(r_dis_sum/cnt_right);
				//printf("r_dis_sum = %f\n", r_dis_sum);
				//printf("average_right = %f\n", average_right); 
				
				right_data = scan->ranges[i]*100;
				if(right_data == 0) right_data = lidar_obstacle_D*100;
			  }  
			}
			
			if( (degree<=90) && (degree>=80) )
			{
			  if(scan->ranges[i] >= 0.1) 
			  {
				cnt_left++;
				//printf("cnt_left = %d\n", cnt_left);	
				l_dis_sum += scan->ranges[i]*100;		
				average_left = (l_dis_sum/cnt_left);
				if(average_left >40.5) average_left = average_left - 40.5;
				//printf("l_dis_sum = %f\n", l_dis_sum); 
				//printf("average_left = %f\n", average_left); 
			  }  			
			}*/ 
			if((degree<=55) && (degree>=10))
				{
				  if(scan->ranges[i] <= 0.9 && scan->ranges[i] >= 0.1 )  // left
				  {
					cnt_right++;
				  // ROS_INFO("cnt_right = %d\n", cnt_right);

					r_dis_sum = r_dis_sum + scan->ranges[i];
			       // lidar_check.data = 1;
					average_right = r_dis_sum/cnt_right;
				//	ROS_INFO("r_dis_sum = %f\n", r_dis_sum);
				    ROS_INFO("average_right = %f\n", average_right); 
				  }
				}
				
			if( (degree<=-10) && (degree>=-55) )
				{
				  if(scan->ranges[i] <= 0.90 && scan->ranges[i] >= 0.10) //RIGHT 
				  {
					cnt_left++;
				//    ROS_INFO("cnt_left = %d\n", cnt_left);
					
					l_dis_sum = l_dis_sum + scan->ranges[i];
					//lidar_check.data = 1;
					average_left = -(l_dis_sum/cnt_left);
				//	ROS_INFO("l_dis_sum = %f\n", l_dis_sum); 
			//		ROS_INFO("average_left = %f\n", average_left); 
				/*if(tuner_flag != 1){
					ROS_INFO("average_left = %f\n", average_left); 
					
				  }*/
			    }
			    
			    
				  /*else if(scan->ranges[i] > 0.1 &&  scan->ranges[i] < 0.2){
				  }	*/
				  /*if(scan->ranges[i] >= 1.0){
							tuner_flag = 1;
							ROS_INFO("tuner_flag = %d\n", tuner_flag); 

						}	*/
				}
		}  
    }
    center = ((average_left+(average_left*0.4)) + average_right)/2 + 0.05; 
    lidar_steer = (center*275.0);
  //  center = (average_left + average_right)/2;
  //  radius = fabs((average_left - average_right)/2);
    tunnel_angle.data = (DEG2RAD(lidar_steer));
    //printf("average_left = %f , average_right = %f \n", average_left, average_right); 
    //printf("center = %.4f\n",center);
    //printf("front = %lf, front2 = %lf, front_right = %lf, right = %lf \n", front_data,front_data2, front_right_data, right_data);
  //  lidar_steer = asin(center/radius);

 //   tunnel_angle.data = lidar_steer;
   //printf("tunnel_angle = %.4f\n", tunnel_angle); 
	//printf("sum = %d\n",sum);
	if (front_data < 25 && front_data > 5) 
	{
		obs_data.data = 1;
	 //   printf("detect!\n");
	  //  printf("front_data %0.4f \n", front_data);
	  //  printf("1obs: %d , 11pass : %d \n",obs_data.data,pass_data.data);
	}
	else	obs_data.data = 0;
	/////////////////////////tunnul/////////////////////////
	if (average_right > 0.1 && average_right < 1.7) 
	{
		obs_4_data.data = 1;
	  //  p/rintf("detect!\n");
	   // printf("obs_4_data: %d \n",tunnel_angle);
	  //  printf("average_right = %0.4f \n",average_right);
	}
	/////////////////////////tunnul/////////////////////////
	else	obs_4_data.data = 0;
	
	if(front_data < 65 && front_data > 15)
	{
		pass_data.data = 1;
	//	printf("detect!\n");
	//	printf("obs: %d , pass : %d \n",obs_data.data,pass_data.data);
	}
	else 	pass_data.data = 0;
	
	
	

	if(front_flag == 0){
		if(front_data < distance)
		{
			front_flag_data.data = 1;
			front_flag = 1;
		}
		else front_flag_data.data = 0;
	}
	
	//  터널구간 
	if(front_data2 > distance2)
	{
		front_flag_data2.data = 1;
	}
	else front_flag_data2.data = 0;
	
	if(front_right_data < 110)
	{
		front_right_flag_data.data = 1;
	}
	else front_right_flag_data.data = 0;

	if(right_data < 75)
	{
		right_flag_data.data = 1;
	}
	else right_flag_data.data = 0;
		
	//printf("front_flag = %d , front_flag2 = %d \n",front_flag_data.data,front_flag_data2.data);
	//printf("front_right_flag = %d , right_flag = %d \n",front_right_flag_data.data, right_flag_data.data);
	
	delete[]obstacle;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "lidar_obstacle_avoid");
	ros::NodeHandle n;

	std::string lidar_topic = "/scan";
	std::string obs_topic = "/lidar/obs_flag";
	std::string obs_4_topic = "/tunnel/obs_flag";
	std::string tunnel_topic = "/lidar/tunnel";
	std::string pass_topic = "/lidar/pass";
	std::string front_topic = "/lidar/front";
	std::string front_right_topic = "/lidar/front_right";
	std::string right_topic = "/lidar/right";
	std::string front2_topic = "/lidar/front2";
	
	ros::param::get("~lidar_topic", lidar_topic);
	ros::param::get("~obs_topic", obs_topic);
	//ros::param::get("~obs_4_topic", obs_4_topic);

	ros::Subscriber sub_lidar = n.subscribe<sensor_msgs::LaserScan>(lidar_topic, 10, &scanCallback);
	ros::Subscriber sub_mission_num = n.subscribe("/mux_mission_flag", 1, mission_Callback);

	ros::Publisher obs_pub = n.advertise<std_msgs::Int8>(obs_topic, 10);
	ros::Publisher obs_4_pub = n.advertise<std_msgs::Int8>(obs_4_topic, 10);
	ros::Publisher tunnel_pub = n.advertise<std_msgs::Float32>(tunnel_topic, 10);
	ros::Publisher pass_pub = n.advertise<std_msgs::Int16>(pass_topic, 10);
	ros::Publisher front_pub = n.advertise<std_msgs::Int16>(front_topic, 10);
	ros::Publisher front_right_pub = n.advertise<std_msgs::Int16>(front_right_topic, 10);
	ros::Publisher right_pub = n.advertise<std_msgs::Int16>(right_topic, 10);
	ros::Publisher front2_pub = n.advertise<std_msgs::Int16>(front2_topic, 10);
		
    ros::Rate loop_rate(10);  // 10
	while (ros::ok())
	{
		
		//printf("mission flag = %d \n",mission_flag);
		//printf("center center = %0.4f \n",center);
		//printf("average_right = %0.4f \n",average_right);
	
		
		if(mission_flag == 2)
		{
			lidar_obstacle_D = LIDAR_PASS;
			pass_pub.publish(pass_data);
		}
		else if(mission_flag == 3){			
			lidar_obstacle_D = LIDAR_Obstacle;
			obs_pub.publish(obs_data);
		}
		else if(mission_flag == 4){
			lidar_obstacle_D = LIDAR_TUNNEL;
			distance = TUNNEL_DISTANCE;	
			//obs_pub.publish(obs_data);////new	
			obs_4_pub.publish(obs_4_data);
			tunnel_pub.publish(tunnel_angle);
			//front_pub.publish(front_flag_data);
		}
		else if(mission_flag == 8)
		{
			lidar_obstacle_D = LIDAR_PARKING;
			distance = PARKING_DISTANCE;
			distance2 = PARKING_DISTANCE2;
			front_right_pub.publish(front_right_flag_data);	
			right_pub.publish(right_flag_data);	
			front_pub.publish(front_flag_data);
			front2_pub.publish(front_flag_data2);
			
		}
		//printf("lidar_obstacle : %lf\n", lidar_obstacle_D);
		loop_rate.sleep();
		ros::spinOnce();
	}

}

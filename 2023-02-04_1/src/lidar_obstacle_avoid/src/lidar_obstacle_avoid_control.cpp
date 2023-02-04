// AA_obstacle_stop.cpp : 이 파일에는 'main' 함수가 포함됩니다. 거기서 프로그램 실행이 시작되고 종료됩니다.
//
#define DEBUG 1

#include "ros/ros.h"
#include "std_msgs/String.h"
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

#include<string.h>
#include "obstacle_processing.h"

#include <iostream>

#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180.*M_PI)

#define MAX_Obstacle_dist 10 //[m]
#define LIDAR_Obstacle   0.1
#define S_LIDAR_Obstacle   0.6
#define O_LIDAR_Obstacle   1.2
#define O_LIDAR_Obstacle_new 1.2
#define LIDAR_Obstacle_angle 10

std_msgs::Int8 obs_data;
std_msgs::Int16 s_course_angle;
std_msgs::Int16 crank_course_angle;
std_msgs::Int16 o_course_r_angle;
std_msgs::Int16 o_course_l_angle;
std_msgs::Int8 direction_data;
std_msgs::Int8 case_8_flag_data;

int mission_flag = 0;
int count = 0;
float lidar_obstacle_D = 0.0;
int direction = 0, pre_angle = 0, error_angle = 0, case_8_flag = 0;

void mission_Callback(const std_msgs::Int16 & msg)
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
	double front_data = lidar_obstacle_D*100;;

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
				sum++;
			}
			
		}
	}
	
	
	if (DEBUG)
	{
		printf("Lidar Obs\n");
		printf("sum = %d,  \n", sum);
		printf("front = %lf,  \n", front_data);
		
	}
	
	if (sum > 25)
	{
		obs_data.data = 1;
	    printf("obs_detect!\n");

	}
	else
	{
		obs_data.data = 0;
	}
	
	delete[]obstacle;

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "lidar_obstacle_avoid");
	ros::NodeHandle n;

	std::string lidar_topic = "/scan";
	std::string obs_topic = "/obs_flag";

	ros::param::get("~lidar_topic", lidar_topic);
	ros::param::get("~obs_topic", obs_topic);

	ros::Subscriber sub_lidar = n.subscribe<sensor_msgs::LaserScan>(lidar_topic, 10, &scanCallback);
	ros::Subscriber sub_mission_num = n.subscribe("/mission_num", 1, mission_Callback);

	ros::Publisher obs_pub = n.advertise<std_msgs::Int8>(obs_topic, 10);
	
    ros::Rate loop_rate(10);  // 10
	while (ros::ok())
	{
		
		lidar_obstacle_D = LIDAR_Obstacle;
		printf("lidar_obstacle : %lf\n", lidar_obstacle_D);

		obs_pub.publish(obs_data);
	
		loop_rate.sleep();
		ros::spinOnce();
	}

}

// AA_obstacle_stop.cpp : 이 파일에는 'main' 함수가 포함됩니다. 거기서 프로그램 실행이 시작되고 종료됩니다.
//
#define DEBUG 1

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float32.h"
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

#include <string.h>
#include "obstacle_processing.h"

#include <iostream>

////////////////


#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180.*M_PI)

#define MAX_Obstacle_dist 10 //[m]
#define LIDAR_Obstacle   0.1
#define S_LIDAR_Obstacle   0.6
#define O_LIDAR_Obstacle   1.2
#define O_LIDAR_Obstacle_new 1.2
#define LIDAR_Obstacle_angle 10

std_msgs::Float32 obs_data;
std_msgs::Int16 s_course_angle;
std_msgs::Int16 crank_course_angle;
std_msgs::Int16 o_course_r_angle;
std_msgs::Int16 o_course_l_angle;
//////////////////
//////////////////
std_msgs::Int16 lidar_check;
////////////////////
std_msgs::Int8 direction_data;
std_msgs::Int8 case_8_flag_data;


int check_case1 = 0;
int mission_flag = 0;
int plus = 0;
int count = 0;
float lidar_obstacle_D = 0.0;
int direction = 0, pre_angle = 0, error_angle = 0, case_8_flag = 0;
/*------------------------lidar---------------------------*/
float center = 0.0;
float lidar_steer = 68.0;

float average_right;
float average_left;

int lidar_flag = 0;
/*------------------------sonar--------------------------*/
float sonar=0.0;

float lidar_sonar = 0.0;
//lidar_check.data = 0;


void mission_Callback(const std_msgs::Int16 & msg)
{
	mission_flag = msg.data;
}
/*
int map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	int count = (int)(360. / RAD2DEG(scan->angle_increment));
	double *obstacle;
	int sum = 0;
	double front_data = lidar_obstacle_D*100;
	

	obstacle = new double[181];
	
	for (int i = 0; i < 181; i++) obstacle[i] = MAX_Obstacle_dist;

	for (int i = 0; i < count; i++)
	{
		int degree = (int)RAD2DEG(scan->angle_min + scan->angle_increment * i);
		printf("degree = %lf,  \n", degree);
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

}*/

////////////////////////////////////////////

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int count = (int)( 360. / RAD2DEG(scan->angle_increment));
    int cnt_right=0;
    int cnt_left =0; 
    int a;
    float r_dis_sum = 0;
    float l_dis_sum = 0;
    ///////////////////////////////////////
    /*if (count == 0)
    {
		lidar_check.data = 0;
	}
	*/
    
	///////////////////////////////////////
    

   // ROS_INFO("I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), count);
   // ROS_INFO("%f %f",scan->scan_time , scan->time_increment);
   // ROS_INFO("angle_range, %f, %f %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max), RAD2DEG(scan->angle_increment));
    /*---------------------------------lidar stop-----------------------------------*/
    for(int i = 0; i < count; i++)
    {	
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        
        if( ((degree>=0) &&(degree<=15)) || ((degree>=345) && (degree<=360)))
        {
          if(scan->ranges[i] <= 0.6) 
          {	
              lidar_sonar = scan->ranges[i];   
             // ROS_INFO("lidar_test = %f\n", lidar_sonar);     
          }  
        }          
    }
    /*---------------------------------lidar steer----------------------------------------*/
    for(int i = 0; i < count; i++)
    {	
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        //////////////////////////flag_3//////////////////////////////
        
        
        switch(lidar_flag) {
			case 0:{
				if((degree>=-10) && (degree<=10))
					{
					  if(scan->ranges[i] <= 0.7 && scan->ranges[i] >= 0.2 && plus < 50) 
					  {
						check_case1 = 1;
						cnt_right++;
						plus++;
						lidar_check.data = 1;
					    ROS_INFO("center_cnt = %d\n", cnt_right);

						r_dis_sum = r_dis_sum + scan->ranges[i];
				
						average_right = r_dis_sum/cnt_right;
					  //  ROS_INFO("center_cnt_sum = %f\n", r_dis_sum);
					  //  ROS_INFO("center_cnt_avg= %f\n", average_right); 
					   ROS_INFO("TURN!!!!!!!!!!\n");
					  }
					 else if(plus >= 50){
							lidar_check.data = 0;
						  }
					  if(check_case1 == 1 && lidar_check.data == 0){
					      lidar_flag = 1;
					  }
					}
					
				}break;
					
			case 1:{
				//ROS_INFO("center_cnt_avg= %f\n", average_right); 
			    //ROS_INFO("GO to home");
				if((degree>=-10) && (degree<=10))
						{
						  if(scan->ranges[i] <= 0.3 && scan->ranges[i] >= 0.2 ) 
						  {
							cnt_right++;
							//ROS_INFO("center_cnt = %d\n", cnt_right);

							r_dis_sum = r_dis_sum + scan->ranges[i];
					
							average_right = r_dis_sum/cnt_right;
						  //  ROS_INFO("center_cnt_sum = %f\n", r_dis_sum);
						  //  ROS_INFO("center_cnt_avg= %f\n", average_right); 
						   ROS_INFO("STOP!!!!!!!!!!\n\n");
						  }  
						}
						break;
					}
			
			case 2:{
				if((degree<=55) && (degree>=10))
				{
				  if(scan->ranges[i] <= 0.6 && scan->ranges[i] >= 0.2 ) 
				  {
					cnt_right++;
				   // ROS_INFO("cnt_right = %d\n", cnt_right);

					r_dis_sum = r_dis_sum + scan->ranges[i];
			
					average_right = r_dis_sum/cnt_right;
				//	ROS_INFO("r_dis_sum = %f\n", r_dis_sum);
				//	ROS_INFO("average_right = %f\n", average_right); 
				  }  
				}
				
				if( (degree<=-10) && (degree>=-55) )
				{
				  if(scan->ranges[i] <= 0.6 && scan->ranges[i] >= 0.2) 
				  {
					cnt_left++;
				   // ROS_INFO("cnt_left = %d\n", cnt_left);
					
					l_dis_sum = l_dis_sum + scan->ranges[i];
					
					average_left = -(l_dis_sum/cnt_left);
				//	ROS_INFO("l_dis_sum = %f\n", l_dis_sum); 
				//	ROS_INFO("average_left = %f\n", average_left); 
				  }  			
				}
				center = (average_left + average_right)/2 + 0.05; 
				lidar_steer = (center*275.0);
				
				if(lidar_steer >= 30)  lidar_steer = 30;
				if(lidar_steer <=-30)  lidar_steer = -30;
				
				obs_data.data = DEG2RAD(lidar_steer);
				//ROS_INFO("lidar_steer = %0.4f\n", DEG2RAD(lidar_steer)); 
				break;
			}
    
    
}

}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "lidar_obstacle_avoid");
	ros::NodeHandle n;

	std::string lidar_topic = "/scan";
	std::string obs_topic = "/obs_flag";
	std::string lidar_check_topic = "/obs_flag_check";
	

	ros::param::get("~lidar_topic", lidar_topic);
	ros::param::get("~obs_topic", obs_topic);
	//ros::param::get("~obs_topic", obs_topic);

	ros::Subscriber sub_lidar = n.subscribe<sensor_msgs::LaserScan>(lidar_topic, 10, &scanCallback);
	ros::Subscriber sub_mission_num = n.subscribe("/mission_num", 1, mission_Callback);

	ros::Publisher obs_pub = n.advertise<std_msgs::Float32>(obs_topic, 10);
	ros::Publisher obs_check_pub = n.advertise<std_msgs::Int16>(lidar_check_topic, 10);
	
    ros::Rate loop_rate(10);  // 10
	while (ros::ok())
	{
		lidar_obstacle_D = LIDAR_Obstacle;
	//	printf("lidar_obstacle : %lf\n", lidar_obstacle_D);

		obs_pub.publish(obs_data);
		obs_check_pub.publish(lidar_check);
		ROS_INFO("check_case1 = %d \n",check_case1);
		ROS_INFO("lidar_check = %d \n",lidar_check.data);
		ROS_INFO("lidar_flag = %d \n",lidar_flag);
		ROS_INFO("Plus = %d \n",plus);
		
	
		loop_rate.sleep();
		ros::spinOnce();
	}

}

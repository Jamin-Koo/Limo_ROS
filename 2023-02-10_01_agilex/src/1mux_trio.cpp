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
#include <cmath>
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
int lidar_case3_flag = 0;
int last_case_flag = 0;
int last_stop_flag = 0;
int case5_flag = 0;
///
int crosswalk_flag = 0;
int stop_line_data = 0;
double tunnel = 0;
int stop_line_flag = 0; 

double linear_x = 0.0, angular_z = 0.0;
double roll,pitch,yaw;
double roll_d,pitch_d,yaw_d, yaw_d_360;
double yaw_d_old  = 0.;
double target_yaw = 0.;
double init_yaw = 0.;
double steer_vel = 42;
double steer_vel_R = 30;
double Kp_y   = 0.01;
double Kd_y   = 0.04;
double Ki_y   = 0.0;

////////chan.../////
int pass = 0;
int tunnel_flag = 0;

double error_y     = 0.0;
double error_d   = 0.0;
double error_sum = 0.0;
//////////////odom///////////////
double position_x = 0.0, position_y = 0.0;
double init_position_x = 0.0, init_position_y = 0.0;
double odom_distance = 0.0;
double avg_x = 0.0; 
double avg_y = 0.0;

///////////////////////////////////////////////
int lidar_detected = 0;
int region_no = 0;
int sub_obs = 0;
int sub_4_obs = 0;

///////////////////////////////////////////////
float no_blob = 0;
int traffic_detect = 0; 
int traffic_flag = 0;
int happy_flag = 0;  

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
  //gion_no = 4;
}

/////////chan..//////
void pass_Callback(const std_msgs::Int16 &msg)
{
	pass = msg.data;
}
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
  position_x = msg->pose.pose.position.x;
  position_y = msg->pose.pose.position.y;
  
  if(odom_init_flag == 1){
	  init_position_x = position_x;
	  init_position_y = position_y;
	  odom_init_flag = 2;
  }
  
  avg_x = position_x - init_position_x;
  avg_y = position_y - init_position_y;
	
	odom_distance = sqrt(pow(avg_x,2)+pow(avg_y,2));
 // ROS_INFO("avg_x = %0.4f avg_y = %0.4f\n",avg_x,avg_y);
  
  
 
  
}

/////////////////lidar////////////////////
void blob_Callback(const std_msgs::Float32 & msg)
{
	no_blob = msg.data;
	//printf("blob data : %d\n", no_blob);
}
//////tunnel/////////
void tunnel_Callback(const std_msgs::Float32 &msg)
{
	tunnel = msg.data;
}

void base_line_xy(void){
	cmd_vel_msg.linear.x = linear_x;
	cmd_vel_msg.angular.z = angular_z;
}

void base_line_zero(void){
	cmd_vel_msg.linear.x = 0;
	cmd_vel_msg.angular.z = 0;
}

void obs_sub_Callback(const std_msgs::Int8 &msg){
	sub_obs = msg.data;
}
void obs_4_sub_Callback(const std_msgs::Int8 &msg){
	sub_4_obs = msg.data;
}
void stopCallback(const std_msgs::Int8 &stop_msg)
{
	stop_line_data = stop_msg.data;
}

double control_yaw(double target_yaw)
{
	double cmd_vel_angluar_z;
	double yaw_d1;

	int error_d = 0;
	double error1;
	double CW_flag = 0.0;
	int     quotient  = 0  ;
	double  remainder = 0.0;

	//double target_yaw = 5.0 ;

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
	
//	ROS_INFO("Yaw Angle : %6.3lf %6.3lf Error : %6.3lf | cmd_vel_angluar_z %6.3lf",yaw_d,yaw,error_y,cmd_vel_angluar_z );
//	ROS_INFO("Target Yaw : %6.3lf Target_Yaw(abs) : %6.3lf ",target_yaw, fabs(target_yaw));
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
	std::string obs_topic = "/lidar/obs_flag";
	std::string obs_4_topic = "/tunnel/obs_flag";
	std::string stop_line_topic = "/stopline_detect";
 
	
	ros::Subscriber lane_cmd_sub = n.subscribe(cmd_vel_lane_topic, 10, lane_Callback);
	ros::Subscriber crosswalk_detect_sub = n.subscribe(sub_crosswalk_detect, 10, crosswalk_detect_Callback);
	ros::Subscriber subIMU = n.subscribe(imu_topic,10, imuCallback);
	ros::Subscriber sub_odom = n.subscribe(odom_topic, 10, odomCallback);
	ros::Subscriber obs_check_sub = n.subscribe(lidar_check_topic, 10, lidar_check_Callback);
	ros::Subscriber region_sub = n.subscribe(region_sub_topic, 10, region_sub_Callback);
	ros::Subscriber obs_sub = n.subscribe(obs_topic, 10, obs_sub_Callback);
	ros::Subscriber obs_4_sub = n.subscribe(obs_4_topic, 10, obs_4_sub_Callback);
	////////chan..//////
	std::string pass_topic = "/lidar/pass";
	ros::Subscriber pass_sub = n.subscribe(pass_topic, 10, pass_Callback);
	
	ros::Subscriber stop_line_sub      = n.subscribe(stop_line_topic, 10, stopCallback);

	
	////tuner////////
	std::string tunnel_topic = "/lidar/tunnel";
	ros::Subscriber tunnel_sub         = n.subscribe(tunnel_topic, 10, tunnel_Callback);
	
	ros::Subscriber sub_blob_no = n.subscribe(blob_topic, 10, blob_Callback);
	
	ros::Publisher  cmd_vel_pub =  n.advertise<geometry_msgs::Twist>(cmd_vel_output_topic, 10);
	ros::Publisher  mux_mission_pub =  n.advertise<std_msgs::Int8>(mux_mission_output_topic, 10);
	
	ros::Rate loop_rate(10.0); //10.0HZ
	//region  0 
	
	 
	while(ros::ok())
  {
	  // pid_control = control_yaw();
	  
	  if(last_case_flag == 0) {
	  
		  switch(region_no) {
			  case 0:{
				 base_line_xy();
				 
				 if(tunnel_flag == 1)
				 {
						cmd_vel_msg.linear.x = 0.0;
			    		cmd_vel_msg.angular.z = 0.0;
						ros::Duration(3.0).sleep();
						last_case_flag = 1;
						odom_init_flag = 1;
				 }
			break;
			}
		  
		  case 1:{
			 pid_control = control_yaw(-95); // +10
			 if(crosswalk_flag == 0){
			    cmd_vel_msg.linear.x = 0.12;
			   	cmd_vel_msg.angular.z = angular_z;
			   	
			   	if(crosswalk_detected == 1) //stop line detect
				{
					cmd_vel_msg.linear.x = 0.0;
					cmd_vel_msg.angular.z = 0.0;
					ros::Duration(2.0).sleep();
				//	printf("Crosswalk Detected, Stop! \n");
					odom_init_flag = 1; // odom reset!
					crosswalk_flag = 1;
				}
				 
				 }
				 
			 else if(crosswalk_flag == 1)
				 {
					 
					if(odom_flag == 0)
					{
						if(odom_distance > 0.18)
						{
							cmd_vel_msg.linear.x = 0.0;
							cmd_vel_msg.angular.z = 0.0;
							ros::Duration(5.0).sleep();
							odom_flag = 1;
						}
						else
						{
							cmd_vel_msg.linear.x = 0.15;
							cmd_vel_msg.angular.z = 0;//pid_control; // yaw angle						
						}
					}
					
					else if(odom_flag == 1){
							
							if(odom_distance > 1.0)
						{
							cmd_vel_msg.linear.x = 0.15;
							cmd_vel_msg.angular.z = angular_z;
						}
						else
						{
							cmd_vel_msg.linear.x = 0.2;
							cmd_vel_msg.angular.z = pid_control; // yaw angle
						}
						
					}
				 
				}
			 
			break;
		}
			
		  case 2:{
		    if(lidar_flag == 0){
				  //pid_control = control_yaw(-148.0); // requrie plz
				  if(pass == 1){
			     // printf("lidar_detected Detected, Stop! \n");
				  odom_init_flag = 1;
				  base_line_zero();
				  ros::Duration(3.0).sleep();
				  lidar_flag = 1;	  
				  }
				  else
				 {
					cmd_vel_msg.linear.x = 0.15;
					cmd_vel_msg.angular.z = angular_z;
				 }
				  
		     }
				  
			 else
			 {
				if(second_odom_flag == 0)
				{
					
					ROS_INFO("odom_flag_0");
					pid_control = control_yaw(11.0+28.0);//
					if(odom_distance > 0.44) //
					{
						cmd_vel_msg.linear.x = 0.0;
						cmd_vel_msg.angular.z = 0.0;
						second_odom_flag = 1;
					}
					else
					{
						cmd_vel_msg.linear.x = 0.2;
						cmd_vel_msg.angular.z = pid_control;
					}
				}
				
				else if(second_odom_flag == 1)
				{
					ROS_INFO("odom_flag_2");
					pid_control = control_yaw(11.0-40.5);
					if(odom_distance > 1.43)
					{
						cmd_vel_msg.linear.x = 0.0;
						cmd_vel_msg.angular.z = 0.0;
						second_odom_flag = 2;
		
					}
					else
					{
						cmd_vel_msg.linear.x = 0.2;
						cmd_vel_msg.angular.z = pid_control;
					}
				}
				
				else if(second_odom_flag == 2){
					ROS_INFO("odom_flag_3");
					cmd_vel_msg.linear.x = linear_x;
					cmd_vel_msg.angular.z = angular_z;
				}
			   
		   }
			
			break;
		}
			
		  
		  case 3:// cha_dan_gi ^^7
		  {
			  //base_line_xy();
	  	   if(sub_obs == 1){
			     printf("chadangi Detected, Stop! \n chadangi Detected, Stop! \nchadangi Detected, Stop! \n");
			  	 base_line_zero();
			    // ros::Duration(3.0).sleep();
			    // lidar_case3_flag = 1;
				  }
   	        else{
					cmd_vel_msg.linear.x = 0.2;
			        cmd_vel_msg.angular.z = angular_z;
			   }
				  
			break;
		 }
		 
		  
		 
		 case 4:
		  {
		  if(sub_4_obs == 1 ){
			     printf("tunnel tunnel, Stop! \n tunnel tunnel, Stop! \tunnel tunnel, Stop! \n");
			  	 cmd_vel_msg.linear.x = 0.2;
			  	 cmd_vel_msg.angular.z = tunnel;
			  	  printf("linear_x = %0.4f " ,linear_x);
				  }
         else{
					cmd_vel_msg.linear.x = 0.2;
			        cmd_vel_msg.angular.z = 0.2;
	                printf("GOGOGOOGOGOGOO \n");

			   }
			
	      break;	  
		 }
		 
		 case 5: { 
			 last_case_flag = 1; 	
	     break;
	}
}
}

else if(last_case_flag != 0){ 
	
	if(thirth_odom_flag == 0){
		 cmd_vel_msg.linear.x = 0.2;
	     cmd_vel_msg.angular.z = angular_z;	
		
		if(stop_line_data == 1){
		    ros::Duration(3.0).sleep();
		    thirth_odom_flag = 1;
		    
		 	}
	
		
		}
	else if(thirth_odom_flag == 1){

		if(stop_line_data == 1){
			pid_control = control_yaw(-95); // +10
			cmd_vel_msg.linear.x = 0.0;
			cmd_vel_msg.angular.z = 0.0;
			ros::Duration(2.0).sleep();
			printf("stopLine, Stop! \n");
			odom_init_flag = 1; // odom reset!
			stop_line_flag = 1;
		    ros::Duration(3.0).sleep();
		    thirth_odom_flag = 2;
		    
		 	}
		 	if(stop_line_flag == 1){
				
		 	if(odom_distance > 1.0)
						{
							cmd_vel_msg.linear.x = 0.2;
							cmd_vel_msg.angular.z = angular_z;
							thirth_odom_flag = 2;
						}
						else
						{
							cmd_vel_msg.linear.x = 0.2;
							cmd_vel_msg.angular.z = pid_control; // yaw angle
						}
					}
			else{
				cmd_vel_msg.linear.x = 0.2;
				cmd_vel_msg.angular.z = angular_z;	
	     
			}
				
		}
		
		else if(thirth_odom_flag == 2){
		 cmd_vel_msg.linear.x = 0.2;
	     cmd_vel_msg.angular.z = angular_z;	
		
		if(stop_line_data == 1){
		    ros::Duration(3.0).sleep();
		    thirth_odom_flag = 3;
		    
		 	}
	
		
		}
		
		
		else if(thirth_odom_flag == 3){
		 cmd_vel_msg.linear.x = 0.2;
	     cmd_vel_msg.angular.z = angular_z;	
		
		if(stop_line_data == 1){
		    ros::Duration(3.0).sleep();
		    thirth_odom_flag = 4;
		    
		 	}
	
		
		}
		
		
		else if(thirth_odom_flag == 4){

		if(stop_line_data == 1){
			pid_control = control_yaw(-95); // +10
			cmd_vel_msg.linear.x = 0.0;
			cmd_vel_msg.angular.z = 0.0;
			ros::Duration(2.0).sleep();
			printf("stopLine, Stop! \n");
			odom_init_flag = 1; // odom reset!
			stop_line_flag = 1;
		    ros::Duration(3.0).sleep();
		    thirth_odom_flag = 2;
		    
		 	}
		 	if(stop_line_flag == 1){
				
		 	            if(odom_distance > 1.0)
						{
							cmd_vel_msg.linear.x = 0.2;
							cmd_vel_msg.angular.z = angular_z;
							thirth_odom_flag = 5;
						}
						else
						{
							cmd_vel_msg.linear.x = 0.2;
							cmd_vel_msg.angular.z = pid_control; // yaw angle
						}
					}
			else{
				cmd_vel_msg.linear.x = 0.2;
				cmd_vel_msg.angular.z = angular_z;	
	     
			}
				
		}
		
		else if(thirth_odom_flag == 5){
		 cmd_vel_msg.linear.x = 0.2;
	     cmd_vel_msg.angular.z = angular_z;	
		
		if(stop_line_data == 1){
		    ros::Duration(3.0).sleep();
		    thirth_odom_flag = 6;
		    
		 	}
	
		
		}
		
		
	/*if(stop_line_data == 1 && thirth_odom_flag == 0){
		base_line_zero();
		ros::Duration(3.0).sleep();
		cmd_vel_msg.linear.x = 0.2;
	    cmd_vel_msg.angular.z = angular_z;	
		odom_init_flag = 1;
		thirth_odom_flag = 1;
		}
		
	else if(stop_line_data == 1 && thirth_odom_flag == 1){ // street
		pid_control = control_yaw(-95); // +10
	    if(crosswalk_flag == 0){
	    cmd_vel_msg.linear.x = 0.12;
	    cmd_vel_msg.angular.z = angular_z;
			   	
	    if(crosswalk_detected == 1) //stop line detect
	 	{
			cmd_vel_msg.linear.x = 0.0;
			cmd_vel_msg.angular.z = 0.0;
			ros::Duration(2.0).sleep();
	   //	printf("Crosswalk Detected, Stop! \n");
			odom_init_flag = 1; // odom reset!
		    crosswalk_flag = 1;
				}
				 
				 }
		
		}
	else if((stop_line_data == 1 && thirth_odom_flag == 2){
		base_line_zero();
		ros::Duration(3.0).sleep();
		cmd_vel_msg.linear.x = 0.2;
	    cmd_vel_msg.angular.z = angular_z;	
		odom_init_flag = 1;
		thirth_odom_flag = 3;
		
		}
		
    else if((stop_line_data == 1 && thirth_odom_flag == 3){
		base_line_zero();
		ros::Duration(3.0).sleep();
		base_line_xy();	
		odom_init_flag = 1;
		thirth_odom_flag = 4;
		
		}
	else if((stop_line_data == 1 && thirth_odom_flag == 4){
		base_line_zero();
		ros::Duration(3.0).sleep();
		base_line_xy();	
		odom_init_flag = 1;
		thirth_odom_flag = 5;
		
		}
	else if((stop_line_data == 1 && thirth_odom_flag == 5){
		base_line_zero();
		ros::Duration(3.0).sleep();
		base_line_xy();	
		odom_init_flag = 1;
		thirth_odom_flag = 6;
		
		}
	if(odom_distance > 2.0)
	{
	    base_line_xy();
	}
	else
	{
		printf("Stop Stop, Stop! \n");
		cmd_vel_msg.linear.x = 0.3;
		pid_control = control_yaw(-90); // +7				
	}
			 
					
		
					
				*/
				
				 
				 
	
}
	  
	mux_mission_msg.data = region_no;
	ROS_INFO("mission = %d \n",region_no);
	ROS_INFO("stop_line_data = %d \n",stop_line_data);
	ROS_INFO("last_case_flag = %d \n",last_case_flag);
	//ROS_INFO("OBS = %d",sub_obs);
	//ROS_INFO("PASS = %d",pass);
	//ROS_INFO("OBS4 = %d \n",sub_4_obs);
	
	
//	ROS_INFO("mission_flag = %d \n",mission_flag);
//	ROS_INFO("odom_flag = %d \n",odom_flag);
//	ROS_INFO("lidar_detected = %d \n",lidar_detected);
//	ROS_INFO( "\n region_no = %d  \n",region_no);
//	ROS_INFO( "\n distance = %d  \n",abs(sqrt(pow(position_x - init_position_x,2) +pow(position_y - init_position_y,2))));
	

	//printf("lidar_detected = %d \n",lidar_detected);
	//ROS_INFO("LIDAR = %0.4f\n",no_blob);
	//ROS_INFO("tunnel = %0.4f\n",tunnel);
	cmd_vel_pub.publish(cmd_vel_msg);
	mux_mission_pub.publish(mux_mission_msg);
	//ROS_INFO("Position-> x: [%f], y: [%f],\n", position_x- init_position_x,position_y -init_position_y );

    ros::spinOnce();
    loop_rate.sleep();
  
}
  return 0;
	
  
}


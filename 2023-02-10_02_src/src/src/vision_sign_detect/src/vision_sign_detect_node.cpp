//opencv_line_detect.cpp
//-I/usr/local/include/opencv4/opencv -I/usr/local/include/opencv4

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Twist.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h> 
#include <string.h> 
#include <stdio.h>
#include <unistd.h>   
#include <stdint.h>   
#include <stdlib.h>  
#include <errno.h>


//multi thread
#include <pthread.h>


using namespace cv;
using namespace std;

typedef unsigned char BYTE;

#define IMG_Width   1280
#define IMG_Height  720

#define USE_DEBUG  1   // 1 Debug  사용
#define USE_CAMERA 1   // 1 CAMERA 사용  0 CAMERA 미사용

#define NO_LINE 20
#define DEG2RAD(x) (M_PI/180.0)*x
#define RAD2DEG(x) (180.0/M_PI)*x
#define Limit_Line_Angle 140

Mat mat_image_org_color;  // Image 저장하기 위한 변수
Mat cv_image;  // Image 저장하기 위한 변수
Mat mat_image_display_color;
Mat mat_image_org_gray;
Mat mat_image_roi_gray;
Mat mat_image_roi_gray_left1;
Mat mat_image_roi_gray_right1;
Mat mat_image_roi_gray_stop_line;
Mat mat_image_roi_threshold_left1;
Mat mat_image_roi_threshold_right1;
Mat mat_image_roi_threshold_stop_line;

Mat mat_image_roi;
Mat mat_image_roi_left1;
Mat mat_image_roi_right1;
Mat mat_image_roi_stop_line;
Mat mat_image_canny_edge;
Mat mat_image_canny_edge_roi;
Mat mat_image_canny_edge_roi_left1;
Mat mat_image_canny_edge_roi_right1;

Scalar GREEN(0,255,0);
Scalar RED(0,0,255);
Scalar BLUE(255,0,0);
Scalar YELLOW(0,255,255);

////////////////////////// Yellow Color Detection ////////////////////////
int iLowH = 20;  int iHighH = 30;
int iLowS = 100;  int iHighS = 255;
int iLowV = 100;  int iHighV = 255;
////////////////////////// Yellow Color Detection ////////////////////////

int  img_width  = 640;
int  img_height = 360;
int  lane_distance = 0;
int  m_roi_center, m_roi_height, m_roi_width,m_roi_width_large;
float  l_line_center, l_line_center_old;
float  r_line_center, r_line_center_old;
int  line_count_r = 0;
int  line_count_l = 0;
int  left_line_detect_flag = 0;
int  right_line_detect_flag = 0;
int  stop_line_detect_status_flag = 0;
int steer_angle_new =0;
int steer_angle_old; 
float lane_center_x = 0.0;
float lane_center_x_old = 0.0;  
   

struct Rect_Region
{
    int left;
	int right;
    int top;
	int bottom;
		
};


struct line_detection_data
{
	
   int x1;
   int y1;
   int x2;
   int y2;
	
};

struct Rect_Region ROI_lane_left1;
struct Rect_Region ROI_lane_right1;
struct Rect_Region ROI_stop_line;
struct Rect_Region ROI_sign_detect;

struct line_detection_data line_data_r[NO_LINE],line_data_l[NO_LINE];
int r_line_intersect[NO_LINE] = {0,};
int l_line_intersect[NO_LINE] = {0,};

Mat Canny_Edge_Detection_r(Mat img);
Mat Canny_Edge_Detection_l(Mat img);
void read_roi_data(void);
Mat Region_of_Interest(Mat image, Point *points);
Mat Region_of_Interest_crop(Mat image, Point *points);


void Init_lane_detect_ROI_l(void);
void Init_lane_detect_ROI_r(void);


void Update_lane_detect_ROI_l(int l_center_x);
void Update_lane_detect_ROI_r(int r_center_x);

void vision_stop_line_detection(void);
void Draw_ROI(void);

int m_roi_center_stop, m_roi_height_stop, m_roi_width_stop;


//////////////////////////////////////// PID control ////////////////////////////////////////////////
float  Kp = 0.5;  float Ki = 0; float  Kd = 0.2;
float  p_gain =0.4; float i_gain=0.2; float d_gain = 0.1;
float  error_lane    = 0; 
float  error_lane_delta    = 0; 
float  error_lane_old = 0;
 
 
 
void read_roi_data(void)
{
	
	FILE *fp;
	int result = 0;
	m_roi_center_stop = m_roi_height_stop = m_roi_width_stop = 0;
	fp = fopen("//home//agilex//AA_data//stop_roi.txt","r");
	
	if(fp == NULL) 
	{
		ROS_INFO("ROI File does not exit ~~ \n\n");
		exit(0);
	}
	
	 
	result = fscanf(fp,"%d %d %d", &m_roi_center_stop, &m_roi_height_stop, &m_roi_width_stop);
	
	ROI_stop_line.top     = m_roi_center_stop - m_roi_height_stop;
	ROI_stop_line.bottom  = m_roi_center_stop + m_roi_height_stop;
	ROI_stop_line.left    = img_width/2 - m_roi_width_stop/2;
	ROI_stop_line.right   = img_width/2 + m_roi_width_stop/2;
	 
	 
	fclose(fp);
	 
	printf("%3d %3d %3d %3d \n\n\n\n",ROI_stop_line.left, ROI_stop_line.top, ROI_stop_line.right , ROI_stop_line.bottom);
	 
}
void lane_detection(void)
{
	Point s_points[4];
	s_points[0] = Point(ROI_stop_line.left,ROI_stop_line.top);	  
    s_points[1] = Point(ROI_stop_line.left,ROI_stop_line.bottom);	  
    s_points[2] = Point(ROI_stop_line.right,ROI_stop_line.bottom);	  
    s_points[3] = Point(ROI_stop_line.right,ROI_stop_line.top);
    mat_image_roi_stop_line = Region_of_Interest_crop(mat_image_org_color,s_points);
    
    mat_image_display_color = mat_image_org_color.clone();
    
    vision_stop_line_detection();
    Draw_ROI();
}
void vision_stop_line_detection(void)
{

	
  int i,j,sum =0;
  int stop_line_image_width;
  int stop_line_image_height;
  
  float occupy_rate;
 
  BYTE *img_buf;
  
 
  cvtColor(mat_image_roi_stop_line, mat_image_roi_gray_stop_line, cv::COLOR_RGB2GRAY);        // color to gray conversion 
  threshold(mat_image_roi_gray_stop_line, mat_image_roi_threshold_stop_line , 100, 255, THRESH_BINARY);
  
  //cout << "Stop Image: " << mat_image_roi_threshold_stop_line;
   
  stop_line_image_width = mat_image_roi_threshold_stop_line.size().width;
  stop_line_image_height = mat_image_roi_threshold_stop_line.size().height;
  
  printf("stop line image size  [%3d | %3d]\n",stop_line_image_width, stop_line_image_height);
  
  
  //img_buf = (BYTE *)malloc(sizeof(BYTE)*stop_line_image_width*stop_line_image_height); 
  //memcpy(img_buf,mat_image_roi_threshold_stop_line.data,(sizeof(BYTE)*stop_line_image_width*stop_line_image_height) );
  
  //cout << "Stop Image: " << img_buf;
  sum = 0;	   
  for(j=0;j < stop_line_image_height; j++)
  {
    for(i=0;i < stop_line_image_width; i++)
    {
      //printf("%d", img_buf[i + (j)*stop_line_image_height]);
		  if( mat_image_roi_threshold_stop_line.at<uchar>(j,i) == 255) sum++;
	}
  }
  //printf("%d\n", sum);
  occupy_rate = (float)sum/(stop_line_image_width*stop_line_image_height);
  
  printf("occupy_rate: %6.3lf\n", occupy_rate);
  if(occupy_rate > 0.25) stop_line_detect_status_flag = 1;
  else                  stop_line_detect_status_flag = 0; 
  printf("stop line pixel count = %d %.1f  %d\n\n", sum, occupy_rate, stop_line_detect_status_flag);
  
  //free(img_buf);
	
	
}

Mat Region_of_Interest_crop(Mat image, Point *points)
{
   Mat img_roi_crop;	
  
   Rect bounds(0,0,image.cols,image.rows);	 
   Rect r(points[0].x, points[0].y, points[2].x-points[0].x, points[2].y-points[0].y);    
  //printf("%d %d %d %d\n",points[0].x,points[0].y,points[2].x-points[0].x, points[2].y-points[0].y);
  // printf("%d  %d\n", image.cols, points[2].y-points[0].y);  
   img_roi_crop = image(r & bounds); 
  
   return img_roi_crop;
}

void Draw_ROI(void)
{
	//printf("w : %3d h : %3d \n",img_width,img_height);

    Rect m_ROI_stop_line;
    m_ROI_stop_line = Rect( Point(ROI_stop_line.left,ROI_stop_line.top), Point(ROI_stop_line.right,ROI_stop_line.bottom));
      
    rectangle(mat_image_display_color,m_ROI_stop_line,Scalar(100,255,100),1,8,0);     
	
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	cv_image = cv_bridge::toCvShare(msg, "bgr8")->image;
	//sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", mat_image_org_color).toImageMsg();
    img_width = cv_image.size().width ;
    img_height = cv_image.size().height;
 
   // printf("w : %3d h : %3d \n",img_width,img_height);
    mat_image_org_color = cv_image.clone();
    lane_detection();
    imshow("view", mat_image_display_color);
    imshow("STOP_IMAGE", mat_image_roi_threshold_stop_line); 
    
    cv::waitKey(30);
}

int main(int argc, char **argv)
{

   std::string cameraTopicName;
   int cameraQueueSize;
   int control_rate	 = 20;     
   	 
   
   std::cout << "OpenCV version : " << CV_VERSION << std::endl;   
   
   read_roi_data();  
   
   ros::init(argc, argv, "vision_sign_detect_node");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
   ros::NodeHandle nh;
   
   nh.param("subscribers/camera_reading/topic", cameraTopicName, std::string("/camera/rgb/image_raw"));
   nh.param("subscribers/camera_reading/queue_size", cameraQueueSize, 1);
   
   ros::Subscriber imageSubscriber = nh.subscribe(cameraTopicName, cameraQueueSize, &imageCallback);
   ros::Publisher stop_linie_detect_status_pub = nh.advertise<std_msgs::Int8>("stopline_detect", 10);
   
   
   
   std_msgs::Int8 stop_line_detect_status_msg;
   stop_line_detect_status_msg.data = 0;
      
   std::cout << "OpenCV version : " << CV_VERSION << std::endl;
  
   ros::Rate loop_rate(20);
   int count = 0;
   int line_count = 0;
   
      
  ////////////////  image display window ///////////////////////////
      
     
   namedWindow("view", WINDOW_NORMAL);
   resizeWindow("Camera Image", IMG_Width/2,IMG_Height/2);
   moveWindow("view", 10, 10);
      
   
   while (ros::ok())
   { 
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    ROS_INFO("-------------  ----- -- -- %d\n", stop_line_detect_status_flag);
    stop_line_detect_status_msg.data = stop_line_detect_status_flag;
    
    stop_linie_detect_status_pub.publish(stop_line_detect_status_msg);
	ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
  
  destroyWindow("view");
  return 0;
}

 

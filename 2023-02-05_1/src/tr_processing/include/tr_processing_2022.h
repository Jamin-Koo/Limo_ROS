#include <iostream>
#include <string>
#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

#include <std_msgs/String.h>
#define SCALE 0.2

using namespace cv;
using namespace std;
using namespace ros;

typedef         unsigned char           u8;
typedef         unsigned short          u16;
typedef         unsigned int            u32;

struct traffic_light
{
	int min;
	int max;
	int distance;
};

class Camera
{
public:
    Camera();
    cv_bridge::CvImagePtr rawImagePtr;
    cv::Mat rawImage;
    cv::Mat rawImage2;
    //void subImgCallback(const sensor_msgs::Image& subImgMsgs); // Camera callback
    void subImgCallback(const sensor_msgs::Image& subImgMsgs); // Camera callback
    void stateCallback(const std_msgs::String& state);
    void msgCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg); // darknet callback
    void find_traffic_light(int class_id);
    void left_tr_light(int class_id);
    void state_publish();
private:
//    ros::NodeHandle nh;
    ros::Subscriber subImage;
    ros::Subscriber boundingbox;
    ros::Subscriber statemsg;
    ros::Publisher pub_traffic_light;
    ros::Publisher err_msgs;
    vector<int> buff;

    string traffic_state = "normal_traffic";

    vector<int> buffer;
};

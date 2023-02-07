#include "tr_processing_2022.h"

string tr_light_map = "left_tr";
string detected_state = "unknown";

Camera::Camera()
{
    ros::NodeHandle nh;
    subImage = nh.subscribe("/usb_cam/image_raw", 1, &Camera::subImgCallback, this);
    boundingbox = nh.subscribe("/darknet_ros/bounding_boxes", 1, &Camera::msgCallback, this);
    statemsg = nh.subscribe("/map_state", 1, &Camera::stateCallback, this);
    pub_traffic_light = nh.advertise<std_msgs::String>("/traffic_light",1000);
    err_msgs = nh.advertise<std_msgs::String>("/error",1000);
}

void Camera::subImgCallback(const sensor_msgs::Image& subImgMsgs)
{
    if(subImgMsgs.data.size())
    {
        rawImagePtr = cv_bridge::toCvCopy(subImgMsgs, sensor_msgs::image_encodings::BGR8);
        rawImage = rawImagePtr->image;

        rawImagePtr->image = rawImage;

        imshow("image", rawImage);
        waitKey(10);
    }
}


void Camera::stateCallback(const std_msgs::String& state)
{
    tr_light_map = state.data;
}


void Camera::find_traffic_light(int class_id)
{
    std_msgs::String msg;
    std::stringstream ss;

    if(class_id==0 or class_id==3 or class_id==4){
        ss<< "go";
        msg.data = ss.str();
        detected_state = ss.str();

        ROS_INFO("%s", msg.data.c_str());
        buffer.clear();
    }
    else if(class_id==1 or class_id==2){
        ss << "stop";
        msg.data = ss.str();
        detected_state = ss.str();

        ROS_INFO("%s", msg.data.c_str());
        buffer.clear();
    }
}

void Camera::left_tr_light(int class_id)
{
    std_msgs::String msg;
    std::stringstream ss;

    if(class_id==3 or class_id==4){
        ss<< "go";
        msg.data = ss.str();
        detected_state = ss.str();

        ROS_INFO("%s", msg.data.c_str());
        buffer.clear();
    }
    else if(class_id==0 or class_id==1 or class_id==2){
        ss << "stop";
        msg.data = ss.str();
        detected_state = ss.str();

        ROS_INFO("%s", msg.data.c_str());
        buffer.clear();
    }
}

void Camera::msgCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{  
    if(rawImage.empty())
    {
        return;
    }
    if(msg->bounding_boxes.size() == 0)
    {
        return;
    }
    
    int max_area = 0;
    int traffic_index = -1;

    for(int i=0; i < msg->bounding_boxes.size(); i++)
    {
        float temp_width = msg->bounding_boxes[i].xmax - msg->bounding_boxes[i].xmin;
        float temp_height = msg->bounding_boxes[i].ymax - msg->bounding_boxes[i].ymin;
        float temp_area = temp_width * temp_height;
    
        if(temp_area >= max_area)
        {
            max_area = temp_area;
            traffic_index = i;
        }
    }

    if(traffic_index == -1){
        return;
    }
    
    if(tr_light_map=="slow_down_for_traffic_light"){
        find_traffic_light(msg->bounding_boxes[traffic_index].id);
    }
    else if(tr_light_map=="left_tr"){
        left_tr_light(msg->bounding_boxes[traffic_index].id);
    }
}

void Camera::state_publish()
{
    std_msgs::String msg;
    
    msg.data = detected_state;
    // ROS_INFO("%s", msg.data.c_str());
    pub_traffic_light.publish(msg);
}
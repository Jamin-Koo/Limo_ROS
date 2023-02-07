#include "traffic_light.h"
#define K_city                     
                                           //DARK       --> need to be set if needed
                                           //BRIGHT
                                           //SIMULATOR
                                           //HALLA

//--> to find the proper value, use 'find_traffic_light_test' in 'ros_image_test'


string detected_state = "unknown";
Camera::Camera()
{
    ros::NodeHandle nh;
    // zoom_state = nh.subscribe("/special_state", 1, &Camera::ZoomStateCallback, this);
    //subImage = nh.subscribe("/image_jpg/compressed", 1, &Camera::subImgCallback, this);
    subImage = nh.subscribe("/traffic_cam_roi/image_raw", 1, &Camera::subImgCallback, this);
    boundingbox = nh.subscribe("/darknet_ros/bounding_boxes", 1, &Camera::msgCallback, this);
    statemsg = nh.subscribe("/map_state", 1, &Camera::stateCallback, this);
    pub_traffic_light = nh.advertise<std_msgs::String>("/traffic_light",1000);
    err_msgs = nh.advertise<std_msgs::String>("/error",1000);
}

void Camera::ZoomStateCallback(const std_msgs::String &state)
{
    if(state.data == "zoom")
    {
        traffic_state = state.data;
    }
    else if(state.data == "glance")
    {
        traffic_state = state.data;
    }
    else if(state.data == "normal_traffic")
    {
        traffic_state = state.data;
    }
}


void Camera::subImgCallback(const sensor_msgs::Image& subImgMsgs)
{

    if(subImgMsgs.data.size())
    {
        rawImagePtr = cv_bridge::toCvCopy(subImgMsgs, sensor_msgs::image_encodings::BGR8);
        rawImage = rawImagePtr->image;

        rawImagePtr->image = rawImage;
        //pubImage.publish(rawImagePtr->toImageMsg());

        // cout<<"rawImage.cols"<<rawImage.cols<<endl;
        // cout<<"rawImage.rows"<<rawImage.rows<<endl;
        // cout<<"mid_point"<<mid_point<<endl<<endl;
        if (traffic_state == "normal_traffic")
        {
            img_width = rawImage.cols;
            img_height = rawImage.rows;
            mid_point = img_width/2;
            x_stretch = img_width/2;
           // mid_point = img_width;
           // x_stretch = img_width;
            // x_stretch = img_width/3;
        }

        else if (traffic_state == "zoom")
        {
            img_width = rawImage.cols;
            img_height = rawImage.rows;
            mid_point = img_width/2;
            x_stretch = 1.5 * img_width/5;
        }

        else if (traffic_state == "glance")
        {
            img_width = rawImage.cols;
            img_height = rawImage.rows;
            x_stretch = img_width/5;
            mid_point = img_width/2 + x_stretch/2;
        }

        line(rawImage, Point(mid_point-x_stretch, 0), Point(mid_point-x_stretch, rawImage.rows), Scalar(100, 100, 100), 2);
	    line(rawImage, Point(mid_point+x_stretch, 0), Point(mid_point+x_stretch, rawImage.rows), Scalar(100, 100, 100), 2);

        imshow("test1", rawImage);
        waitKey(10);
    }
}


void Camera::stateCallback(const std_msgs::String& state)
{
    //cout<< "stateCallback_TEST"<<endl;
    std_msgs::String msg;
    std::stringstream ss;
    ss << "slow_down_for_traffic_light";
    msg.data = ss.str();

    if(state.data != msg.data)
    {
        detected_state = "unknown";
        
        buffer.clear();
    }
}


void Camera::find_traffic_light(Mat frame)
{
    std_msgs::String msg;
    std::stringstream ss;

    Mat HSV;
    vector<Mat> origin_channel_HSV;
    vector<Mat> origin_channel_BGR;

    cvtColor(frame, HSV, COLOR_BGR2HSV);
    split(HSV, origin_channel_HSV);
    split(frame, origin_channel_BGR);

   
    
    Mat red_area1;
    Mat red_area2;
    Mat red_area;
    Mat green_area;

    #ifdef DARK
    //little dark (afternoon)
    //===============================================================================================================
    inRange(HSV, Scalar(0, 158, 48), Scalar(60, high_S, high_V), red_area1);
    inRange(HSV, Scalar(150, 0, 40), Scalar(180, high_S, high_V), red_area2);
    inRange(HSV, Scalar(45, 158, 48), Scalar(95, high_S, high_V), green_area);
    //===============================================================================================================
    #endif

    #ifdef BRIGHT
    //bright (morning)
    //===============================================================================================================
    inRange(HSV, Scalar(0, 150, 48), Scalar(40, high_S, high_V), red_area1); //25
    inRange(HSV, Scalar(150, 48, 40), Scalar(180, high_S, high_V), red_area2);
    inRange(HSV, Scalar(45, 158, 48), Scalar(95, high_S, high_V), green_area);
    //===============================================================================================================
    #endif

    #ifdef SIMULATOR
    //simulator
    //===============================================================================================================
    inRange(HSV, Scalar(0, 50, 50), Scalar(50, high_S, high_V), red_area1);
    inRange(HSV, Scalar(130, 50, 50), Scalar(180, high_S, high_V), red_area2);
    inRange(HSV, Scalar(50, 50, 48), Scalar(75, high_S, high_V), green_area);
    //===============================================================================================================
    #endif

    #ifdef HALLA
    //halla university
    //===============================================================================================================
    inRange(HSV, Scalar(0, 150, 48), Scalar(60, high_S, high_V), red_area1);
    inRange(HSV, Scalar(150, 50, 50), Scalar(180, high_S, high_V), red_area2);
    inRange(HSV, Scalar(45, 100, 48), Scalar(95, high_S, high_V), green_area);
    //===============================================================================================================
    #endif


    #ifdef K_city
    //halla university
    //===============================================================================================================
    inRange(HSV, Scalar(10, 100, 40), Scalar(45, high_S, high_V), red_area1);
    inRange(HSV, Scalar(140, 50, 40), Scalar(220, high_S, high_V), red_area2);
    inRange(HSV, Scalar(85, 150, 40), Scalar(105, high_S, high_V), green_area);
    //===============================================================================================================
    #endif

    bitwise_or(red_area1, red_area2, red_area);

    Mat light, green_stats, red_stats, centroid; 
    int green_numlabel = connectedComponentsWithStats(green_area, light, green_stats, centroid, 8, CV_32S);
    int red_numlabel = connectedComponentsWithStats(red_area, light, red_stats, centroid, 8, CV_32S);
    
    int gmax_area = 0;
    int rmax_area = 0;

    if(green_numlabel > 1 && red_numlabel > 1){
        for(int i=1; i < green_numlabel; i++) {
            if(gmax_area < green_stats.at<int>(i, CC_STAT_AREA)){
                gmax_area = green_stats.at<int>(i, CC_STAT_AREA);
            }
        }
        for(int i=1; i < red_numlabel; i++) {
            if(rmax_area < red_stats.at<int>(i, CC_STAT_AREA)){
                rmax_area = red_stats.at<int>(i, CC_STAT_AREA);
            }
        }
        if(gmax_area > rmax_area){
            ss<< "GREEN";
            msg.data = ss.str();
            detected_state = ss.str();

            ROS_INFO("%s", msg.data.c_str());
            buffer.clear();
        }
        else{
            ss<< "RED";
            msg.data = ss.str();
            detected_state = ss.str();

            ROS_INFO("%s", msg.data.c_str());
            buffer.clear();
        }
    }
    else if(green_numlabel>1){
        ss << "GREEN";
        msg.data = ss.str();
        detected_state = ss.str();

        ROS_INFO("%s", msg.data.c_str());
        buffer.clear();
    }
    else if(red_numlabel>1){
        ss << "RED";
        msg.data = ss.str();
        detected_state = ss.str();

        ROS_INFO("%s", msg.data.c_str());
        buffer.clear();
    }
    else{
        cout << "error" << endl;
    }
    
    imshow("HSV", HSV);
    imshow("red_area1", red_area1);
    imshow("red_area2", red_area2);
    imshow("red_area", red_area);
    imshow("green_area", green_area);

    waitKey(10);

}

//get BoundingBoxes area from darknet
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
    int max_size = -1;
    int max_index = 0;

    int traffic_index = -1;
    int tr_lt_num = 0;

    for(int i = 0 ; i < msg->bounding_boxes.size(); i++)
    {
        if(msg->bounding_boxes[i].Class == "traffic light")
        {
            if((msg->bounding_boxes[i].xmin >= mid_point-x_stretch) && (msg->bounding_boxes[i].xmax <= mid_point+x_stretch))
            {
                // float temp_width = msg->bounding_boxes[i].xmax - msg->bounding_boxes[i].xmin;
                // float temp_height = msg->bounding_boxes[i].ymax - msg->bounding_boxes[i].ymin;
                // float temp_ratio = temp_width / temp_height;
                // if(temp_ratio >= 2.5 && temp_ratio <= 3.7)
                // {
                //     traffic_index = i;
                //     break;
                // }
                traffic_index = i;
            }
        }
    }

    if(traffic_index == -1){
        return;
    }

    int temp_xmin = msg->bounding_boxes[traffic_index].xmin;
    int temp_xmax = msg->bounding_boxes[traffic_index].xmax;
    int temp_ymin = msg->bounding_boxes[traffic_index].ymin;
    int temp_ymax = msg->bounding_boxes[traffic_index].ymax;

    //========================================================================================
    if(temp_xmax <= 0 || temp_xmin > img_width || temp_ymax <= 0 || temp_ymin > img_height || temp_ymin <= 0 || temp_xmin <= 0 ||temp_xmax <= temp_xmin|| temp_ymax <= temp_ymin)
    {
        if(re_xmax != 0 && re_ymax != 0)
        {
            temp_xmin = re_xmin;
            temp_xmax = re_xmax;
            temp_ymin = re_ymin;
            temp_ymax = re_ymax;
        }
        else
        {
            return;
        }
    }
    else
    {
        re_xmin = temp_xmin;
        re_xmax = temp_xmax;
        re_ymin = temp_ymin;
        re_ymax = temp_ymax;     
    }
    

    int xmin = temp_xmin;
    int xmax = temp_xmax;
    int ymin = temp_ymin;
    int ymax = temp_ymax;
    //========================================================================================

    int width = xmax - xmin;
    int height = ymax - ymin;

    float w_h_Ratio = width / height;
   
    #ifdef SIMULATOR
    w_stretch = width * 0.5 + 4 * width * 0.5/3;
    h_stretch = height + 3 * height/3;
    #endif

    #ifndef SIMULATOR
    w_stretch = width * 0.5;
    h_stretch = height;
    #endif

    // if(xmin - w_stretch < 0)
    // {
    //     w_stretch = 0;
    // }
    // else if(xmax + w_stretch >= rawImage.cols)
    // {
    //     w_stretch = rawImage.cols - xmax;
    // }

    // if(ymin - h_stretch <0)
    // {
    //     h_stretch = 0;
    // }
    // else if(ymax + h_stretch >= rawImage.rows)
    // {
    //     h_stretch = rawImage.cols - ymax;
    // }

    Rect roi(xmin - w_stretch, ymin - h_stretch, xmax-xmin + 2*w_stretch, ymax-ymin + 2*h_stretch);
    
    // Rect roi(xmin, ymin, width, height);

    if(roi.x < 0)
        roi.x = 0;

    if(roi.y < 0)
        roi.y = 0;

    if(roi.x + roi.width >= rawImage.cols)
    {
        int offset = roi.x + roi.width - rawImage.cols;
        roi.width = roi.width - offset - 1;
    }

    if(roi.y + roi.height >= rawImage.rows)
    {
        int offset = roi.y + roi.height - rawImage.rows;
        roi.height = roi.height - offset - 1;
    }

    

    #ifdef SIMULATOR
    Rect sim_roi = Rect(roi.x + roi.width/3 , roi.y + roi.height/3, roi.width - 2 * roi.width/3, roi.height - 2 * roi.height/3 /*roi.x , roi.y, roi.width, roi.height*/);
    Mat frame = rawImage(sim_roi);
    //rectangle(rawImage, Point(sim_roi.x, sim_roi.y), Point(sim_roi.x + sim_roi.width, sim_roi.y + sim_roi.height), Scalar(0, 200, 0), 2);
    
    resize(frame, frame, Size(roi.width,roi.height), 0, 0, CV_INTER_LINEAR);
    imshow("frame",frame);
    waitKey(10);
    find_traffic_light(frame);
    #endif

    #ifndef SIMULATOR
    Mat frame = rawImage(roi);
    imshow("frame",frame);
    waitKey(10);
    find_traffic_light(frame);
    #endif

    min_y = (1024)/2;   //initialize
    
}
void Camera::state_publish()
{
    std_msgs::String msg;
    
    msg.data = detected_state;
    // ROS_INFO("%s", msg.data.c_str());
    pub_traffic_light.publish(msg);
}
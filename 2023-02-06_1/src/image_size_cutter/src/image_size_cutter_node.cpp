#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Header.h"
#include "std_msgs/String.h"
#include "image_transport/image_transport.h"
#include <opencv2/highgui/highgui.hpp>
#include "cv_bridge/cv_bridge.h"
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

cv_bridge::CvImagePtr cv_ptr;
cv_bridge::CvImagePtr filtered_cv_ptr;
cv::Mat Image;
cv::Mat Image_roi;
sensor_msgs::ImagePtr Img_msg;
int width;
int height;
bool small_trl = false;


void CutterCallback(const sensor_msgs::Image::ConstPtr &img)
{
    ROS_INFO("Image(%d, %d)", img->width, img->height);

    width = img->width;
    height = img->height;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

// void ZoomCallback(const std_msgs::String &zoom_msg)
// {
//     if(zoom_msg.data == "zoom")
//     {
//         small_trl = true;
//     }
//     else if (zoom_msg.data == "no_zoom")
//     {
//         small_trl = false;
//     }

//     cout << "small_trl = " << small_trl << endl;
// }


int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_size_cutter");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);

    image_transport::Publisher pub_img_roi;
    image_transport::Subscriber sub_img;
    image_transport::Publisher pub_img_2zoom;

    ros::Rate loop_rate(5);

    sub_img = it.subscribe("/camera/rgb/image_raw", 1, CutterCallback); 
    // sub_img = it.subscribe("/camera1/image_jpeg/raw", 1, CutterCallback); 
    // sub_img = it.subscribe("camera1/traffic_cam_prcd/image_raw", 1, CutterCallback); 
    // sub_img = it.subscribe("/camera1/traffic_cam/image_raw", 1, CutterCallback);//for GO pro (edit Joo)
    pub_img_roi = it.advertise("/traffic_cam_roi/image_raw",1000);
    //sub_img = it.subscribe("/image_jpg/compressed", 1, CutterCallback);
    // pub_img_2zoom = it.advertise("/camera1/traffic_cam_for_zoom/image_raw",1000);
    // ros::Subscriber state_sub = nh.subscribe("/zoom_state",1,ZoomCallback);

    while(ros::ok())
    {
        if (!cv_ptr)
        {
            std::cout<<"Waiting for image."<<std::endl;
        }
        
        if(cv_ptr)
        {
            Image = cv_ptr->image;

            // Rect roi = Rect(rawImage.cols / 4, rawImage.rows / 4, rawImage.cols / 2, rawImage.rows / 2);
            // Mat dst = rawImage(roi);
            // cv::resize(dst, dst, cv::Size(dst.cols * 2, dst.rows * 2), 0, 0, CV_INTER_LINEAR);

            // msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dst).toImageMsg();
            // last = dst;

            // // cv::Rect roi(0, 200, width, height-200);    // car_num_detection
            cv::Rect roi(100, 0, width-100, (height-300));     // traffic_light
            // cv::Rect roi(500, 0, width/2, height/2);
            // // cv::Rect roi(0, 0, width, height);
            // Image_roi = Image(roi);


            

            // cv::Mat smaller;
            // cv::Mat bigger;
            // cv::Mat laplacian;

            // // 원본 영상을 가우시안 피라미드로 축소
            // cv::pyrDown(Image, smaller);
            // cv::pyrUp(smaller, bigger);

            // // 원본에서 확대한 영상 빼기
            // cv::subtract(Image, bigger, laplacian);
            // // 확대 한 영상에 라플라시안 영상 더해서 복원
            // cv::Mat restored = bigger + laplacian;

            cv::Mat dst = Image(roi);
            // cv::resize(dst, dst, cv::Size(dst.cols * 2, dst.rows * 2), 0, 0, CV_INTER_CUBIC);
            // cv::resize(dst, dst, cv::Size(), 2, 2, CV_INTER_LINEAR);

            // cv::Mat dst2clahe;
            // cv::createCLAHE(dst, dst2clahe);



            Img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dst).toImageMsg();

            // cv::imshow("Image Show_2", dst2);
            // cv::imshow("Image Show", Image_roi);
            // 결과 출력 (원본 영상, 라플라시안, 확대 영상, 복원 영상)
            // cv::imshow("Laplacian Pyramid", restored);
            cv::imshow("dst Show", dst);
            cv::waitKey(1);
            pub_img_roi.publish(Img_msg);


            //cv_ptr.reset();
        }
        else
        {
           ; 
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

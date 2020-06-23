#include "ros/ros.h"
#include <time.h>
#include <iostream>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

using namespace std;
using namespace cv;

class CameraSwitcher{
private:
    ros::NodeHandle nh_;
    ros::Subscriber camera1_image_sub_;
    ros::Subscriber camera2_image_sub_;

    ros::Publisher image_pub_;

    sensor_msgs::Image image_;
    
    time_t start_, end_;
    double result_;
    bool image1_, image2_;
    bool first_frame_;
    bool image1_error_;

    Mat frame_;

    //For Optical Flow
    vector<Scalar> colors_;
    vector<Point2f> p0_, p1_;
//    vector<Point2f> good_new_;
//    vector<uchar> status_;
//    vector<float> err_;

    Mat prev_frame_, prev_gray_, old_gray_;
    Mat frame_gray_;
    Mat mask_;

    TermCriteria criteria_;

public:
    CameraSwitcher();
    ~CameraSwitcher();

    void parseRawing(const sensor_msgs::ImageConstPtr& ros_img, cv::Mat& cv_img);

    void image1_cb(const sensor_msgs::ImageConstPtr& image);
    void image2_cb(const sensor_msgs::ImageConstPtr& image);
    void initsetup();
    void run();

    void OpticalFlowLK();

};

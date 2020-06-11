#include "ros/ros.h"
#include <time.h>
#include <iostream>
#include <string>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

using namespace std;

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
public:
    CameraSwitcher();
    ~CameraSwitcher();

    void image1_cb(const sensor_msgs::ImageConstPtr& image);
    void image2_cb(const sensor_msgs::ImageConstPtr& image);
    void initsetup();
    void run();


};

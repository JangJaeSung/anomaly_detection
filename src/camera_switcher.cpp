#include "anomaly_detection/camera_switcher.h"

using namespace std;

CameraSwitcher::CameraSwitcher():nh_("~")
{
    initsetup();
}

CameraSwitcher::~CameraSwitcher()
{
    initsetup();
}

void CameraSwitcher::initsetup()
{
    cout << "start init" << endl;
    image1_ = true;
    image2_ = false;
    camera1_image_sub_ = nh_.subscribe("/usb_cam1/image_raw", 100, &CameraSwitcher::image1_cb, this);
    camera2_image_sub_ = nh_.subscribe("/usb_cam2/image_raw", 100, &CameraSwitcher::image2_cb, this);
}

void CameraSwitcher::image1_cb(const sensor_msgs::ImageConstPtr& image)
{
    image_pub_ = nh_.advertise<sensor_msgs::Image>("/usb_cam/image_raw", 1);

    cv_bridge::CvImagePtr cam_image;
    cv_bridge::CvImage cvImage;

    try{
        cam_image = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8); 
    }catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cvImage.header.stamp = ros::Time::now();
    cvImage.header.frame_id = "camera";
    cvImage.encoding = sensor_msgs::image_encodings::BGR8;
    cvImage.image = cam_image->image;
    
    if (image1_)
        image_pub_.publish(*cvImage.toImageMsg());
}

void CameraSwitcher::image2_cb(const sensor_msgs::ImageConstPtr& image)
{
    image_pub_ = nh_.advertise<sensor_msgs::Image>("/usb_cam/image_raw", 1);

    cv_bridge::CvImagePtr cam_image;
    cv_bridge::CvImage cvImage;

    try{
        cam_image = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8); 
    }catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cvImage.header.stamp = ros::Time::now();
    cvImage.header.frame_id = "camera";
    cvImage.encoding = sensor_msgs::image_encodings::BGR8;
    cvImage.image = cam_image->image;

    if (image2_)
        image_pub_.publish(*cvImage.toImageMsg());
}

void CameraSwitcher::run()
{
    ros::Rate r(60);
    start_ = time(0);
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
        end_ = time(0);
        result_ = (double)(end_ - start_);
        cout << result_ << endl;
        if (result_ == 5) {
            image1_ == true ? ((image1_ = false), (image2_ = true)) : ((image1_ = true), (image2_ = false));
            result_ = 0;
            start_ = time(0);
        }
        cout << image1_ << image2_ << endl;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_switcher");
    cout << "node start" << endl;
    CameraSwitcher camera_switcher;
    camera_switcher.run();

    return 0;
}


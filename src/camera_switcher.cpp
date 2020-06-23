#include "anomaly_detection/camera_switcher.h"

using namespace std;
using namespace cv;

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
    first_frame_ = false;
    image1_error_ = false;
    camera1_image_sub_ = nh_.subscribe("/usb_cam1/image_raw", 100, &CameraSwitcher::image1_cb, this);
    camera2_image_sub_ = nh_.subscribe("/usb_cam2/image_raw", 100, &CameraSwitcher::image2_cb, this);
}


void CameraSwitcher::parseRawing(const sensor_msgs::ImageConstPtr& ros_img, cv::Mat& cv_img)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(ros_img, sensor_msgs::image_encodings::BGR8);

    cv_img = cv_ptr->image;

    if (cv_img.empty()){
        throw runtime_error("The frame is empty");
    }
}

void CameraSwitcher::OpticalFlowLK()
{
    // Refresh corner point
    if (result_ == 3){ 
        goodFeaturesToTrack(prev_gray_, p0_, 100, 0.3, 7, Mat(), 7, false, 0.04);
    }

    cvtColor(frame_, frame_gray_, COLOR_BGR2GRAY);

    vector<uchar> status;
    vector<float> err;
    criteria_ = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 10, 0.03);
    calcOpticalFlowPyrLK(prev_gray_, frame_gray_, p0_, p1_, status, err, Size(15, 15), 2, criteria_);

    vector<Point2f> good_new;
    for(uint i = 0; i < p0_.size(); i++){
        if (status[i] == 1){
            good_new.push_back(p1_[i]);
            //line(mask_, p1_[i], p0_[i], colors_[i], 2);
            circle(frame_, p1_[i], 5, colors_[i], -1);
        }
    }

    // For drawing vector points

    //Mat print_img;
    //add(frame_, mask_, print_img);

//    for (int i = 0; i < status.size(); i++){
//        cout << err.at(i) << ' ';  
//        printf("%u ", status.at(i));
//    }
//    printf("\n");

    imshow("Frame", frame_);
    waitKey(3);
    prev_gray_ = frame_gray_.clone();
    p0_ = good_new;
}


void CameraSwitcher::image1_cb(const sensor_msgs::ImageConstPtr& image)
{
    image_pub_ = nh_.advertise<sensor_msgs::Image>("/usb_cam/image_raw", 1);

    //cv_bridge::CvImagePtr cam_image;
    cv_bridge::CvImage cvImage;

    try{
        parseRawing(image, frame_); 
    }catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }catch(const runtime_error& e){
        cerr << e.what() << endl;
    }
    if (!first_frame_){
        cvtColor(frame_, old_gray_, COLOR_BGR2GRAY);
        goodFeaturesToTrack(old_gray_, p0_, 100, 0.3, 7, Mat(), 7, false, 0.04);

        mask_ = Mat::zeros(frame_.size(), frame_.type());
        prev_gray_ = old_gray_;

        first_frame_ = true;
        cout << "first frame" << endl;
    }
    else if (first_frame_ && !image1_error_){
        try{
            OpticalFlowLK();
        }catch (cv::Exception& e){
            image1_error_ = true;
            cv::destroyAllWindows();
            ROS_ERROR("Image error occur -> camera changed ");
        }
    }

    cvImage.header.stamp = ros::Time::now();
    cvImage.header.frame_id = "camera";
    cvImage.encoding = sensor_msgs::image_encodings::BGR8;
    cvImage.image = frame_;
    
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

    RNG rng;
    for (int i = 0; i < 100; i++){
        int r = rng.uniform(0, 256);
        int g = rng.uniform(0, 256);
        int b = rng.uniform(0, 256);
        colors_.push_back(Scalar(r, g, b));
    }

   while(ros::ok()){
        end_ = time(0);
        result_ = (double)(end_ - start_);

        ros::spinOnce();
        r.sleep();
       cout << result_ << endl;
        if (result_ == 3){
            result_ = 0;
            start_ = time(0);
        }
        if (image1_error_) {
            //image1_ == true ? ((image1_ = false), (image2_ = true)) : ((image1_ = true), (image2_ = false));
            image1_ = false;
            image2_ = true;
            //result_ = 0;
            //start_ = time(0);
        }
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


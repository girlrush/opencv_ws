#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>

#pragma once

class ImageProc
{
private:
    ros::NodeHandle nh;

    ros::Publisher pub_procImg_canny;
    ros::Publisher pub_procImg_hough;
    ros::Publisher pub_procImg_keypoint;
    ros::Subscriber sub_image;

    int gaussian_min; //0~360
    int gaussian_max; //0~360

    cv::Mat std_image;
    cv::Mat std_descriptors;
    std::vector<cv::KeyPoint> std_keypoint;
    cv::Ptr<cv::Feature2D> std_f2d = cv::xfeatures2d::SURF::create();  


    void ReadParam(ros::NodeHandle& node_);
    void cb_image(const sensor_msgs::ImageConstPtr& msg);

public:
    ImageProc(){ }

    ~ImageProc(){ }

    void init(ros::NodeHandle& nh);
};


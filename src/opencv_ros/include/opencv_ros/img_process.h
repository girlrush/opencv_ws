#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

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
    
    // ros::Subscriber sub_left_image;
    // ros::Subscriber sub_right_image;

    int gaussian_min; //0~360
    int gaussian_max; //0~360

    cv::Mat std_image;
    cv::Mat std_descriptors;
    std::vector<cv::KeyPoint> std_keypoint;
    cv::Ptr<cv::Feature2D> std_f2d = cv::xfeatures2d::SURF::create();  

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_policy_stereo;
    typedef std::shared_ptr<message_filters::Synchronizer<sync_policy_stereo>> synchronizer_stereo;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> sub_left_image;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> sub_right_image;

    synchronizer_stereo sync_stereo;

    void ReadParam(ros::NodeHandle& node_);
    // void cb_image(const sensor_msgs::ImageConstPtr& msg);

    void CallbackStereo(const sensor_msgs::ImageConstPtr& , const sensor_msgs::ImageConstPtr&);

public:
    ImageProc(){ }

    ~ImageProc(){ }

    void init(ros::NodeHandle& nh);
};


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

struct ParamStereo{
    int num_disparities     = 8;
    int block_size          = 5;
    int pre_filter_type     = 1;
    int pre_filter_size     = 1;
    int pre_filter_cap      = 31;
    int min_disparity       = 0;
    int texture_threshold   = 10;
    int uniqueness_ratio    = 15;
    int speckle_range       = 0;
    int speckle_window_size = 0;
    int disp12_max_diff     = -1;
    int dis_type            = CV_16S;
};

class ImageProc
{
private:
    ros::NodeHandle nh;

    ros::Publisher pub_canny;
    ros::Publisher pub_hough;
    ros::Publisher pub_disparity;

    ros::Publisher pub_left_nice;
    ros::Publisher pub_right_nice;

    
    int gaussian_min; //0~360
    int gaussian_max; //0~360


    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_policy_stereo;
    typedef std::shared_ptr<message_filters::Synchronizer<sync_policy_stereo>> synchronizer_stereo;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> sub_left_image;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> sub_right_image;

    synchronizer_stereo sync_stereo;

    ParamStereo param_stereo;
    cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create();

    void ReadParam(ros::NodeHandle& node_);
    void CallbackStereo(const sensor_msgs::ImageConstPtr& , const sensor_msgs::ImageConstPtr&);

public:
    ImageProc(){ }

    ~ImageProc(){ }

    void init(ros::NodeHandle& nh);
};


#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

#pragma once

class ImageProc
{
private:
    ros::NodeHandle nh;

    ros::Publisher pub_procImg;
    ros::Subscriber sub_image;

    int gaussian_min; //0~360
    int gaussian_max; //0~360


    void ReadParam(ros::NodeHandle& node_);
    void cb_image(const sensor_msgs::ImageConstPtr& msg);

public:
    ImageProc(){ }

    ~ImageProc(){ }

    void init(ros::NodeHandle& nh);
};


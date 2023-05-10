#include <img_process.h>

void ImageProc::init(ros::NodeHandle& node_)
{
    nh = node_;
    
    pub_procImg = nh.advertise<sensor_msgs::Image>("output", 10);
    sub_image = nh.subscribe<sensor_msgs::Image>("camera/color/image_raw", 10, &ImageProc::cb_image, this);

    ReadParam(node_);
}

void ImageProc::ReadParam(ros::NodeHandle& node_)
{
    if(!node_.getParam("gaussian_min", gaussian_min))
        ROS_WARN("[Fail] get - gaussain_min");
    else
        ROS_INFO("[Param] gaussain_min : %d", gaussian_min);


    if(!node_.getParam("gaussian_max", gaussian_max))
        ROS_WARN("[Fail] get - gaussain_max");
    else
        ROS_INFO("[Param] gaussain_max : %d", gaussian_max);

    
}

void ImageProc::cb_image(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_bridge::CvImage cv_proc;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        return;
    }

    cv::GaussianBlur(cv_ptr->image, cv_proc.image, cv::Size(3,3), 0);
    cv::Canny(cv_proc.image, cv_proc.image, gaussian_min, gaussian_max);


    cv_proc.encoding = sensor_msgs::image_encodings::MONO8;

    pub_procImg.publish(cv_proc.toImageMsg());
}   

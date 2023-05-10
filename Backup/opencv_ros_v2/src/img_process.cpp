#include <img_process.h>

void ImageProc::init(ros::NodeHandle& node_)
{
    nh = node_;
    
    pub_procImg = nh.advertise<sensor_msgs::Image>("output", 10);
    pub_procImg_keypoint = nh.advertise<sensor_msgs::Image>("output/keypoint", 10);
    sub_image = nh.subscribe<sensor_msgs::Image>("camera/color/image_raw", 10, &ImageProc::cb_image, this);

    ReadParam(node_);

    std::cout << "standard image process" << std::endl;
    std_image = cv::imread("/root/opencv_ws/src/opencv_ros/frame1473.jpg", 0);
    
    std::cout << "standard image size : " << std_image.size() << std::endl;

    cv::GaussianBlur(std_image, std_image, cv::Size(3,3), 0);

    cv::Canny(std_image, std_image, gaussian_min, gaussian_max);

    std_f2d->detect(std_image, std_keypoint);
    std_f2d->compute(std_image, std_keypoint, std_descriptors);

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
    std::cout << "Start image callback" << std::endl;
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

    std::cout << "Image Size : " << cv_ptr->image.size() << std::endl;

    cv::GaussianBlur(cv_ptr->image, cv_proc.image, cv::Size(3,3), 0);

    cv::Canny(cv_proc.image, cv_proc.image, gaussian_min, gaussian_max);

    cv::Ptr<cv::Feature2D> f2d = cv::xfeatures2d::SURF::create();  

    std::vector<cv::KeyPoint> keypoint;
    f2d->detect(cv_proc.image, keypoint);

    cv::Mat descriptors;
    f2d->compute(cv_proc.image, keypoint, descriptors);

    cv_bridge::CvImage cv_proc_with_keypoint;
    cv::drawKeypoints(cv_proc.image, keypoint, cv_proc_with_keypoint.image, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    cv_proc_with_keypoint.encoding = sensor_msgs::image_encodings::MONO8;
    pub_procImg_keypoint.publish(cv_proc_with_keypoint.toImageMsg());

    cv_proc.encoding = sensor_msgs::image_encodings::MONO8;
    pub_procImg.publish(cv_proc.toImageMsg());

    



    // // 특징점 비교
    // cv::BFMatcher matcher;
    // std::vector<cv::DMatch> matches;
    // matcher.match(descriptors, std_descriptors, matches);
    // cv::Mat dst;
    // cv::drawMatches(std_image, std_keypoint, cv_proc.image, keypoint, matches, dst);
    // imshow("dst", dst);


    // cv_proc.encoding = sensor_msgs::image_encodings::MONO8;

    // pub_procImg.publish(cv_proc.toImageMsg());
}   

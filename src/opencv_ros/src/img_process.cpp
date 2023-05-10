#include <img_process.h>

void ImageProc::init(ros::NodeHandle& node_)
{
    nh = node_;
    
    ReadParam(node_);

    pub_procImg_canny = nh.advertise<sensor_msgs::Image>("/output/canny", 10);
    pub_procImg_hough = nh.advertise<sensor_msgs::Image>("/output/hough", 10);

    pub_procImg_keypoint = nh.advertise<sensor_msgs::Image>("/output/keypoint", 10);

    sub_image = nh.subscribe<sensor_msgs::Image>("/image_topic", 10, &ImageProc::cb_image, this);


    //std::cout << "standard image process" << std::endl;
    //std_image = cv::imread("/root/opencv_ws/src/opencv_ros/frame1473.jpg", 0);
    
    //std::cout << "standard image size : " << std_image.size() << std::endl;

    //cv::GaussianBlur(std_image, std_image, cv::Size(3,3), 0);

    //cv::Canny(std_image, std_image, gaussian_min, gaussian_max);

    //std_f2d->detect(std_image, std_keypoint);
    //std_f2d->compute(std_image, std_keypoint, std_descriptors);

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
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    }
    catch (cv_bridge::Exception& e)
    {
        return;
    }

    std::cout << "Image Size : " << cv_ptr->image.size() << std::endl;

    cv::cvtColor(cv_ptr->image, cv_proc.image, cv::COLOR_RGB2GRAY);
    cv::GaussianBlur(cv_proc.image, cv_proc.image, cv::Size(3,3), 0);

    cv::Canny(cv_proc.image, cv_proc.image, gaussian_min, gaussian_max);

    cv::Ptr<cv::Feature2D> f2d = cv::xfeatures2d::SURF::create();  


    cv_proc.encoding = sensor_msgs::image_encodings::MONO8;
    pub_procImg_canny.publish(cv_proc.toImageMsg());

    std::vector<cv::Vec4i> lines;

    cv::HoughLinesP(cv_proc.image, lines, 1, CV_PI/180, 80, 10, 250);

    // cv::Mat img_temp(cv_ptr->image.size(), CV_8UC3, cv::Scalar(0,0,0));
    cv::Mat img_temp = cv_ptr->image.clone();
    
    for (std::size_t i=0; i<lines.size(); i++)
    {
        cv::Vec4i l = lines[i];
        cv::line(img_temp, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    }

    cv_proc.encoding = sensor_msgs::image_encodings::RGB8;
    cv_proc.image = img_temp.clone();
    pub_procImg_hough.publish(cv_proc.toImageMsg());

}   

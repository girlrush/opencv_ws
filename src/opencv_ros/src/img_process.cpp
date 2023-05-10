#include <img_process.h>

void ImageProc::init(ros::NodeHandle& node_)
{
    nh = node_;
    
    ReadParam(node_);

    pub_procImg_canny = nh.advertise<sensor_msgs::Image>("/output/canny", 10);
    pub_procImg_hough = nh.advertise<sensor_msgs::Image>("/output/hough", 10);

    pub_procImg_keypoint = nh.advertise<sensor_msgs::Image>("/output/keypoint", 10);

    // sub_left_image = nh.subscribe<sensor_msgs::Image>("/left_image_topic", 10, &ImageProc::cb_image, this);
    // sub_right_image = nh.subscribe<sensor_msgs::Image>("/right_image_topic", 10, &ImageProc::cb_image, this);

    sub_left_image.reset(new message_filters::Subscriber<sensor_msgs::Image>(nh, "/stereo/left", 10));
    sub_right_image.reset(new message_filters::Subscriber<sensor_msgs::Image>(nh, "/stereo/right", 10));

    sync_stereo.reset(new message_filters::Synchronizer<sync_policy_stereo>(sync_policy_stereo(100), *sub_left_image, *sub_right_image));
    sync_stereo->registerCallback(boost::bind(&ImageProc::CallbackStereo, this, _1, _2));

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

void ImageProc::CallbackStereo(const sensor_msgs::ImageConstPtr& img_left, const sensor_msgs::ImageConstPtr& img_right)
{
    
}

// void ImageProc::cb_image(const sensor_msgs::ImageConstPtr& msg)
// {
//     std::cout << "Start image callback" << std::endl;
//     cv_bridge::CvImagePtr cv_ptr;
//     cv_bridge::CvImage cv_proc;

//     try
//     {
//         cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
//     }
//     catch (cv_bridge::Exception& e)
//     {
//         return;
//     }

//     std::cout << "Image Size : " << cv_ptr->image.size() << std::endl;

//     cv::cvtColor(cv_ptr->image, cv_proc.image, cv::COLOR_RGB2GRAY);
//     cv::GaussianBlur(cv_proc.image, cv_proc.image, cv::Size(3,3), 0);

//     cv::Canny(cv_proc.image, cv_proc.image, gaussian_min, gaussian_max);

//     cv::Ptr<cv::Feature2D> f2d = cv::xfeatures2d::SURF::create();  


//     cv_proc.encoding = sensor_msgs::image_encodings::MONO8;
//     pub_procImg_canny.publish(cv_proc.toImageMsg());

//     std::vector<cv::Vec4i> lines;

//     cv::HoughLinesP(cv_proc.image, lines, 1, CV_PI/180, 80, 10, 250);

//     // cv::Mat img_temp(cv_ptr->image.size(), CV_8UC3, cv::Scalar(0,0,0));
//     cv::Mat img_temp = cv_ptr->image.clone();
    
//     for (std::size_t i=0; i<lines.size(); i++)
//     {
//         cv::Vec4i l = lines[i];
//         cv::line(img_temp, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
//     }

//     cv_proc.encoding = sensor_msgs::image_encodings::RGB8;
//     cv_proc.image = img_temp.clone();
//     pub_procImg_hough.publish(cv_proc.toImageMsg());

// }   

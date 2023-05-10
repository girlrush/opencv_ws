#include <img_process.h>

void ImageProc::init(ros::NodeHandle& node_)
{
    nh = node_;
    
    ReadParam(nh);

    pub_canny = nh.advertise<sensor_msgs::Image>("/output/canny", 10);
    pub_hough = nh.advertise<sensor_msgs::Image>("/output/hough", 10);
    pub_disparity = nh.advertise<sensor_msgs::Image>("/output/disparity", 10);

    pub_left_nice  = nh.advertise<sensor_msgs::Image>("/output/left_nice", 10);
    pub_right_nice = nh.advertise<sensor_msgs::Image>("/output/right_nice", 10);

    sub_left_image.reset(new message_filters::Subscriber<sensor_msgs::Image>(nh, "/stereo/left", 10));
    sub_right_image.reset(new message_filters::Subscriber<sensor_msgs::Image>(nh, "/stereo/right", 10));

    sync_stereo.reset(new message_filters::Synchronizer<sync_policy_stereo>(sync_policy_stereo(100), *sub_left_image, *sub_right_image));
    sync_stereo->registerCallback(boost::bind(&ImageProc::CallbackStereo, this, _1, _2));
    

}

void ImageProc::ReadParam(ros::NodeHandle& node_)
{

    nh.param("gaussian/min", gaussian_min, 0);
    nh.param("gaussian/max", gaussian_max, 360);


    std::cout << "Gaussian max = " << gaussian_max << std::endl;
    std::cout << "Gaussian min = " << gaussian_min << std::endl;
    
}

void ImageProc::CallbackStereo(const sensor_msgs::ImageConstPtr& img_left, const sensor_msgs::ImageConstPtr& img_right)
{
    cv_bridge::CvImagePtr left_cv_ptr;
    cv_bridge::CvImagePtr right_cv_ptr;

    cv_bridge::CvImage left_cv_proc;
    cv_bridge::CvImage right_cv_proc;

    cv_bridge::CvImage disparity_cv_proc;

    try
    {
        // Convert sensor_msgs to CvImage
        left_cv_ptr  = cv_bridge::toCvCopy(img_left, sensor_msgs::image_encodings::RGB8);
        right_cv_ptr = cv_bridge::toCvCopy(img_right, sensor_msgs::image_encodings::RGB8);

        std::cout << "Left image : " << left_cv_ptr->image.size() << std::endl;
        std::cout << "Right image : " << right_cv_ptr->image.size() << std::endl;
    }
    catch (cv_bridge::Exception& e) { std::cout << "Fail to convert images" << std::endl; return;}

    // Change color to gray scale
    cv::cvtColor (left_cv_ptr->image, left_cv_proc.image, cv::COLOR_RGB2GRAY);
    cv::cvtColor (right_cv_ptr->image, right_cv_proc.image, cv::COLOR_RGB2GRAY);

    left_cv_ptr->encoding = sensor_msgs::image_encodings::MONO8;
    right_cv_ptr->encoding = sensor_msgs::image_encodings::MONO8;

    cv::Mat disp(left_cv_ptr->image.size(), CV_32FC1), disparity(left_cv_ptr->image.size(), CV_32FC1);

    cv::Mat left_stereo_map1(left_cv_ptr->image.size(), CV_32FC1), left_stereo_map2(left_cv_ptr->image.size(), CV_32FC1);
    cv::Mat right_stereo_map1(right_cv_ptr->image.size(), CV_32FC1), right_stereo_map2(right_cv_ptr->image.size(), CV_32FC1);


    cv::Mat left_nice, right_nice;

    cv::remap(left_cv_proc.image,
        left_nice,
        left_stereo_map1,
        left_stereo_map2,
        cv::INTER_LANCZOS4,
        cv::BORDER_CONSTANT,
        0);


    cv::remap(right_cv_proc.image,
        right_nice,
        right_stereo_map1,
        right_stereo_map2,
        cv::INTER_LANCZOS4,
        cv::BORDER_CONSTANT,
        0);
    
    // stereo->compute(left_nice, right_nice, disp);
    stereo->compute(left_cv_proc.image, right_cv_proc.image, disp);

    disp.convertTo(disparity, CV_8UC1);
    
    cv::normalize(disparity, disparity, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    
    // disp.convertTo(disparity, CV_32F, 1.0);
    // disparity = (disparity/16.0f - (float)param_stereo.min_disparity / (float)param_stereo.num_disparities);

    disparity.copyTo(disparity_cv_proc.image);
    disparity_cv_proc.encoding = sensor_msgs::image_encodings::MONO8;

    pub_disparity.publish(disparity_cv_proc);



    // cv_bridge::CvImage left_nice_cv;
    // cv_bridge::CvImage right_nice_cv;

    // left_nice.copyTo(left_nice_cv.image);
    // right_nice.copyTo(right_nice_cv.image);
    
    // left_nice_cv.encoding = sensor_msgs::image_encodings::RGB8;
    // right_nice_cv.encoding = sensor_msgs::image_encodings::RGB8;

    // pub_left_nice.publish(left_nice_cv);
    // pub_right_nice.publish(right_nice_cv);


    left_nice.copyTo(left_cv_proc.image);
    right_nice.copyTo(right_cv_proc.image);

    pub_left_nice.publish(left_cv_proc);
    pub_right_nice.publish(right_cv_proc);
    
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

#include <img_process.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "opencv_node");
    ros::NodeHandle nh;

    ROS_INFO("Node Start");

    ImageProc img_node;
    img_node.init(nh);

    ros::spin();
}
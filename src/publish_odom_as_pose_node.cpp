#include <publish_odom_as_pose/publish_odom_as_pose.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "publish_odom_as_pose_node");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros_utils::PublishOdomAsPose publish_odom_as_pose(nh, private_nh);

    ros::spin();
    return 0;
}

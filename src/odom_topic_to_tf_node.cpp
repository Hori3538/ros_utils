#include <odom_topic_to_tf/odom_topic_to_tf.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_topic_to_tf_node");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros_utils::OdomTopicToTF odom_topic_to_tf(nh, private_nh);

    ros::spin();
    return 0;
}

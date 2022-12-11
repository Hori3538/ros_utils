#include <odom_transformer/odom_transformer.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_transformer_node");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros_utils::OdomTransformer odom_transformer(nh, private_nh);

    ros::spin();
    return 0;
}

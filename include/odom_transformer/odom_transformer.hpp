#ifndef ODOM_TRANSFORMER
#define ODOM_TRANSFORMER

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace ros_utils
{
    struct Param
    {
        std::string odom_topic_name;
    };

    class OdomTransformer
    {
        public:
            OdomTransformer(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
            ~OdomTransformer();
        private:
            void odom_callback(const nav_msgs::OdometryConstPtr &msg);
            void transform_odom(nav_msgs::Odometry &odom);
            
            Param param_;

            ros::Subscriber odom_sub_;
            ros::Publisher odom_transformed_pub_;

            tf2_ros::Buffer tf_buffer_;
            tf2_ros::TransformListener* tf2_listener_;
    };
}

#endif


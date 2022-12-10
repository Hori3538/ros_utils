#ifndef ODOM_TOPIC_TO_TF
#define  ODOM_TOPIC_TO_TF

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
// #include <tf/>
#include <tf2_ros/transform_broadcaster.h>

#include <optional>

namespace ros_utils
{
    struct Param
    {
        std::string odom_topic_name;
        int hz;
    };

    class OdomTopicToTF
    {
        public:
            OdomTopicToTF(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
            void process();
        private:
            void odom_callback(const nav_msgs::OdometryConstPtr &msg);
            geometry_msgs::TransformStamped odom_to_tf(nav_msgs::Odometry odom);

            Param param_;
            std::optional<nav_msgs::Odometry> odom_;
            // nav_msgs::Odometry odom_;

            ros::Subscriber odom_sub_;
    };
}

#endif 

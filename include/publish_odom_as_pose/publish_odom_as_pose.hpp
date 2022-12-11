#ifndef PUBLISH_ODOM_AS_POSE
#define PUBLISH_ODOM_AS_POSE

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

namespace ros_utils
{
    struct Param
    {
        std::string odom_topic_name;
    };

    class PublishOdomAsPose
    {
        public:
            PublishOdomAsPose(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
        private:
            void odom_callback(const nav_msgs::OdometryConstPtr &msg);
            geometry_msgs::PoseStamped odom_to_pose(nav_msgs::Odometry odom);
            
            Param param_;

            ros::Subscriber odom_sub_;
            ros::Publisher odom_pose_pub_;
    };
}

#endif


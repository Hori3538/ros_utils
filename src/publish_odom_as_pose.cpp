#include <publish_odom_as_pose/publish_odom_as_pose.hpp>

namespace ros_utils
{
    PublishOdomAsPose::PublishOdomAsPose(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
    {
        private_nh.param<std::string>("odom_topic_name", param_.odom_topic_name, "whill/odom");

        odom_sub_ = nh.subscribe<nav_msgs::Odometry>(param_.odom_topic_name, 10, &PublishOdomAsPose::odom_callback, this);
        odom_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>(param_.odom_topic_name + "/pose", 1);

    }

    void PublishOdomAsPose::odom_callback(const nav_msgs::OdometryConstPtr &msg)
    {
        nav_msgs::Odometry odom = *msg;
        geometry_msgs::PoseStamped odom_pose = odom_to_pose(odom);
        odom_pose_pub_.publish(odom_pose);
    }

    geometry_msgs::PoseStamped PublishOdomAsPose::odom_to_pose(nav_msgs::Odometry odom)
    {
        geometry_msgs::PoseStamped odom_pose;
        odom_pose.header = odom.header;
        odom_pose.pose.position = odom.pose.pose.position;
        odom_pose.pose.orientation = odom.pose.pose.orientation;

        return odom_pose;
    }
}

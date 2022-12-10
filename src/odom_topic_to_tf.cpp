#include "geometry_msgs/TransformStamped.h"
#include <odom_topic_to_tf/odom_topic_to_tf.hpp>

namespace ros_utils
{
    OdomTopicToTF::OdomTopicToTF(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
    {
        private_nh.param<std::string>("odom_topic_name", param_.odom_topic_name, "whill/odom");
        private_nh.param<int>("hz", param_.hz, 50);

        odom_sub_ = nh.subscribe<nav_msgs::Odometry>(param_.odom_topic_name, 1, &OdomTopicToTF::odom_callback, this);
    }

    void OdomTopicToTF::odom_callback(const nav_msgs::OdometryConstPtr &msg)
    {
        odom_ = *msg;
    }
    
    geometry_msgs::TransformStamped OdomTopicToTF::odom_to_tf(nav_msgs::Odometry odom)
    {
        geometry_msgs::TransformStamped odom_state;
        odom_state.header = odom.header;
        // odom_state.header.stamp = ros::Time::now();
        odom_state.child_frame_id = "base_link";

        odom_state.transform.translation.x = odom.pose.pose.position.x;
        odom_state.transform.translation.y = odom.pose.pose.position.y;
        odom_state.transform.translation.z = odom.pose.pose.position.z;

        odom_state.transform.rotation = odom.pose.pose.orientation;

        return odom_state;
    }

    void OdomTopicToTF::process()
    {
        tf2_ros::TransformBroadcaster odom_state_broadcaster;
        ros::Rate loop_rate(param_.hz);

        while (ros::ok())
        {
            if(odom_.has_value())
            {
                geometry_msgs::TransformStamped odom_state = odom_to_tf(odom_.value());
                odom_state_broadcaster.sendTransform(odom_state);

            }
            ros::spinOnce();
            loop_rate.sleep();
        }

    }
}

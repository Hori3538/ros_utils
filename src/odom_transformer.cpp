#include <odom_transformer/odom_transformer.hpp>

namespace ros_utils
{
    OdomTransformer::OdomTransformer(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
    {
        private_nh.param<std::string>("odom_topic_name", param_.odom_topic_name, "whill/odom");

        odom_sub_ = nh.subscribe<nav_msgs::Odometry>(param_.odom_topic_name, 50, &OdomTransformer::odom_callback, this);
        odom_transformed_pub_ = nh.advertise<nav_msgs::Odometry>(param_.odom_topic_name + "/transformed", 1);

        tf2_listener_ = new tf2_ros::TransformListener(tf_buffer_);
    }

    OdomTransformer::~OdomTransformer()
    {
        delete this->tf2_listener_;
    }

    void OdomTransformer::odom_callback(const nav_msgs::OdometryConstPtr &msg)
    {
        nav_msgs::Odometry odom = *msg;
        transform_odom(odom);
        odom_transformed_pub_.publish(odom);
    }

    void OdomTransformer::transform_odom(nav_msgs::Odometry &odom)
    {
        nav_msgs::Odometry odom_transformed;
        geometry_msgs::PoseStamped odom_pose;
        odom_pose.header = odom.header;
        odom_pose.pose = odom.pose.pose;

        try
        {
            geometry_msgs::TransformStamped transform;
            transform = tf_buffer_.lookupTransform(odom.child_frame_id, odom.header.frame_id, ros::Time(0));
            tf2::doTransform(odom_pose, odom_pose, transform);
            odom.child_frame_id = odom.header.frame_id;
            odom.header = odom_pose.header;
            odom.pose.pose = odom_pose.pose;
        }
        catch(tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
        }
    }
}

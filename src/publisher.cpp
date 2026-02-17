#include "optitrack4px4/publisher.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

Publisher::Publisher(std::string topic_name, rclcpp::Node *node)
{
    position_publisher_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(topic_name, 10);

    // Pose Euler publisher with same topic name + "_euler" suffix
    rpy_publisher_ = node->create_publisher<mocap_msgs::msg::PoseEuler>(topic_name + "_euler", 10);

    is_ready = true;
}

void Publisher::publish(geometry_msgs::msg::PoseStamped pose_msg)
{
    // Publish PoseStamped (quaternion)
    position_publisher_->publish(pose_msg);

    // Convert quaternion to roll/pitch/yaw and publish as PoseEuler
    tf2::Quaternion q(
        pose_msg.pose.orientation.x,
        pose_msg.pose.orientation.y,
        pose_msg.pose.orientation.z,
        pose_msg.pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    mocap_msgs::msg::PoseEuler euler_msg;
    euler_msg.header = pose_msg.header;
    euler_msg.x = pose_msg.pose.position.x;
    euler_msg.y = pose_msg.pose.position.y;
    euler_msg.z = pose_msg.pose.position.z;
    euler_msg.roll = roll;
    euler_msg.pitch = pitch;
    euler_msg.yaw = yaw;
    rpy_publisher_->publish(euler_msg);
}

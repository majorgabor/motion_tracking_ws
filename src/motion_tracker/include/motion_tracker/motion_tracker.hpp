#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "tf2_ros/static_transform_broadcaster.h"

#ifndef _MOTION_TRACKER_H_
#define _MOTION_TRACKER_H_

class MotionTracker : public rclcpp::Node
{
    std::vector<nav_msgs::msg::Odometry> estimated_odometries_;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr estimated_odometry_pub_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr incoming_imu_data_subs_;

    void incomingImuDataCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

    void publishStaticTransform();

public:
    MotionTracker();
};

#endif
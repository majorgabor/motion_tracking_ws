#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#ifndef _PUBLISHER_H_
#define _PUBLISHER_H_

#define G_FORCE 9.8

class PublisherNode : public rclcpp::Node
{
    sensor_msgs::msg::Imu m_;
    char time_buff_[10], decimal_buff_[7];

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;

public:
    PublisherNode();

    void publish(const char * const buffer);
};

#endif
#include <socket_to_ros_topic/publisher_node.hpp>

PublisherNode::PublisherNode() : Node("publisher_node")
{
    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
}

void PublisherNode::publish(const char * const buffer)
{
    memcpy(&time_buff_, buffer, 10);
    m_.header.stamp = rclcpp::Time((int64_t)atoi(time_buff_) * 1e3);
    
    memcpy(&decimal_buff_, buffer + 10, 5);
    m_.linear_acceleration.x = G_FORCE * atof(decimal_buff_);
    memcpy(&decimal_buff_, buffer + 15, 5);
    m_.linear_acceleration.y = G_FORCE * atof(decimal_buff_);
    memcpy(&decimal_buff_, buffer + 20, 5);
    m_.linear_acceleration.z = G_FORCE * atof(decimal_buff_);
    
    memcpy(&decimal_buff_, buffer + 25, 7);
    m_.angular_velocity.x = atof(decimal_buff_) * 0.01745;
    memcpy(&decimal_buff_, buffer + 32, 7);
    m_.angular_velocity.y = atof(decimal_buff_) * 0.01745;
    memcpy(&decimal_buff_, buffer + 39, 7);
    m_.angular_velocity.z = atof(decimal_buff_) * 0.01745;

    publisher_->publish(m_);
}
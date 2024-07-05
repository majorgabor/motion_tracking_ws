# Motion Tracking WS

This is a IMU based 3D motion tracking project, implemented using C++, Python, and ROS2.

Tis project able to track motion from IMU data arriving on it's interface (UDP socket or http polling).

The motion tracking algorithm is extremely simple, it is able to track motion _okey-ish_ in the first 1-3 seconds then ofc the drift makes it unusable.

### csv_to_rosbag

Simple ROS2 package to convert IMU data from CSV format (exported from 'phyphox') to ROS2 bag format.

### http_to_ros_topic

Polls a HTTP address for IMU data then publish it as ROS2 message. Use it with 'phyphox' to read sensor data from mobile phone.

### motion_tracker

The ROS2 node, that implements the motion tracking algorithm.

### socket_to_ros_topic

An UDP client, which reads incoming data then publish it as ROS2 message.
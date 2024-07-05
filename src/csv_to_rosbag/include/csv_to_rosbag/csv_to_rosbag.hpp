#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <csv_to_rosbag/vector3.hpp>

#ifndef _CSV_TO_ROSBAG_H_
#define _CSV_TO_ROSBAG_H_

class CsvToRosbag : public rclcpp::Node
{
    
    std::unique_ptr<rosbag2_cpp::Writer> writer_;
    void writeToBag(const std::vector<double> &time_stamps, const std::vector<Vector3> &linear_acc, const std::vector<Vector3> &angular_acc);

public:
    CsvToRosbag();
    
};

#endif
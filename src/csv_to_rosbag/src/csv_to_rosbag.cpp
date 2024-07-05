#include <csv_to_rosbag/csv_to_rosbag.hpp>
#include <csv_to_rosbag/csv_reader.hpp>
#include <sensor_msgs/msg/imu.hpp>

CsvToRosbag::CsvToRosbag() : Node("csv_to_rosbag")
{
    // declare params
    this->declare_parameter("input_csv_files_folder", rclcpp::PARAMETER_STRING);
    this->declare_parameter("output_bag_file", rclcpp::PARAMETER_STRING);

    // read params
    std::string param_input_csv_files_folder, param_output_bag_file;
    if (!get_parameter<std::string>("input_csv_files_folder", param_input_csv_files_folder))
    {
        RCLCPP_FATAL(get_logger(), "Could not retrieve parameter: input_csv_files_folder");
        throw std::runtime_error("failed to load parameter");
    }
    if (!get_parameter<std::string>("output_bag_file", param_output_bag_file))
    {
        RCLCPP_FATAL(get_logger(), "Could not retrieve parameter: output_bag_file");
        throw std::runtime_error("failed to load parameter");
    }

    // setup rosbag
    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    writer_->open(param_output_bag_file);
    writer_->create_topic({"/record",
                           "sensor_msgs/msg/Imu",
                           rmw_get_serialization_format(),
                           "",
                           ""});

    // read accelerations from csv
    const std::string linear_acceleration_csv_file{param_input_csv_files_folder + "/Linear Accelerometer.csv"};
    const std::string angular_acceleration_csv_file{param_input_csv_files_folder + "/Gyroscope.csv"};

    auto linear_accelerations = readAccelerationFromScv(linear_acceleration_csv_file);
    auto angular_accelerations = readAccelerationFromScv(angular_acceleration_csv_file);

    // check if timestamps align
    if (linear_accelerations.first.size() != angular_accelerations.first.size())
    {
        RCLCPP_FATAL(get_logger(), "Linear and angular acceleration data length do NOT match!");
        throw std::runtime_error("linear and angular acceleration data length do not match");
    }
    for (size_t i{0lu}; i < linear_accelerations.first.size(); i++)
    {
        if (linear_accelerations.first[i] != angular_accelerations.first[i])
        {
            RCLCPP_FATAL(get_logger(), "Timestamps i:%lu do NOT match!", i);
            throw std::runtime_error("timestamps do not match");
        }
    }

    RCLCPP_INFO(get_logger(), "record list size: %lu", linear_accelerations.first.size());

    writeToBag(linear_accelerations.first, linear_accelerations.second, angular_accelerations.second);
}

void CsvToRosbag::writeToBag(const std::vector<double> &time_stamps, const std::vector<Vector3> &linear_acc, const std::vector<Vector3> &angular_acc)
{
    sensor_msgs::msg::Imu data;
    for (size_t i{0ul}; i < time_stamps.size(); i++)
    {
        rclcpp::Time time_i_in_rclcpp_format = rclcpp::Time(static_cast<uint64_t>(time_stamps[i] * 1e9)); // convert to nanosec
        data.header.stamp = time_i_in_rclcpp_format;
        data.linear_acceleration.x = linear_acc[i].x;
        data.linear_acceleration.y = linear_acc[i].y;
        data.linear_acceleration.z = linear_acc[i].z;
        data.angular_velocity.x = angular_acc[i].x;
        data.angular_velocity.y = angular_acc[i].y;
        data.angular_velocity.z = angular_acc[i].z;

        writer_->write<sensor_msgs::msg::Imu>(data, "/record", time_i_in_rclcpp_format);
    }
}
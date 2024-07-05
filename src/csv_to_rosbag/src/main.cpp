#include <csv_to_rosbag/csv_to_rosbag.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    try
    {
        executor.add_node(std::make_shared<CsvToRosbag>());
        executor.spin_once();
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
    rclcpp::shutdown();

    return 0;
}
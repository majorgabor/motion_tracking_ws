#include <motion_tracker/motion_tracker.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotionTracker>());
    rclcpp::shutdown();
    
    return 0;
}
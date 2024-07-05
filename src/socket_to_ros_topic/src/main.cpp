#include <socket_to_ros_topic/udp_socket_server.hpp>
#include <socket_to_ros_topic/signal_handler.hpp>
#include <socket_to_ros_topic/publisher_node.hpp>
#include <memory>
#include <csignal>

int main(int argc, char** argv)
{
    std::signal(SIGINT, signalHandler);
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::SigTerm);

    std::shared_ptr<UdpSocketServer> udp_socket_server = std::make_shared<UdpSocketServer>(8888, 46);
    std::shared_ptr<PublisherNode> publisher_node = std::make_shared<PublisherNode>();

    while (!stop_signal)
    {
        udp_socket_server->receiveMessage();
        publisher_node->publish(udp_socket_server->getBuffer());
    }

    rclcpp::shutdown();

    return 0;
}
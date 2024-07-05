#include <socket_to_ros_topic/udp_socket_server.hpp>
#include <sys/socket.h>
#include <unistd.h>
#include <rclcpp/logging.hpp>

UdpSocketServer::UdpSocketServer(const uint16_t port, const size_t buffer_size) : socket_id_(socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)), buffer_size_(buffer_size)
{
    if (socket_id_ < 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("my_logger"), "Socket creation failed!");
        exit(1);
    }

    memset(&server_address_, 0, sizeof(server_address_));
    server_address_.sin_family = AF_INET; // IPv4
    server_address_.sin_addr.s_addr = INADDR_ANY;
    server_address_.sin_port = htons(port);

    if (bind(socket_id_, (const struct sockaddr *)&server_address_, sizeof(server_address_)) < 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("my_logger"), "Bind failed!");
        exit(1);
    }

    buffer_ptr_ = new char[buffer_size];
}

UdpSocketServer::~UdpSocketServer()
{
    close(socket_id_);
    delete buffer_ptr_;
}

void UdpSocketServer::receiveMessage()
{
        recv(socket_id_, buffer_ptr_, buffer_size_, 0);
}

const char * const UdpSocketServer::getBuffer() const
{
    return buffer_ptr_;
}
#include <netinet/in.h>
#include <string>

#ifndef _UDP_SOCKET_SERVER_H_
#define _UDP_SOCKET_SERVER_H_

class UdpSocketServer
{
    const int socket_id_;
    char *buffer_ptr_;
    const size_t buffer_size_;
    sockaddr_in server_address_;


public:
    UdpSocketServer(const uint16_t port, const size_t buffer_size);
    ~UdpSocketServer();

    void receiveMessage();

    const char * const getBuffer() const;
};

#endif
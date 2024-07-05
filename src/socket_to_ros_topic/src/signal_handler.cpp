#include <socket_to_ros_topic/signal_handler.hpp>

void signalHandler(int)
{
    stop_signal = true;
}



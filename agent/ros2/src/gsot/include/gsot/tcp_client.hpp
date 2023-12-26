#ifndef TCP_CLIENT_HPP
#define TCP_CLIENT_HPP

#include <string>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <functional>
#include <rclcpp/rclcpp.hpp>

class TCPClient {
    public:
        TCPClient(const std::string& ip, uint16_t port, rclcpp::Logger logger);
        void sendData(const char *buffer, size_t buffer_size) const;
        void receiveData(std::function<void(const std::string &)> callback) const;
        ~TCPClient();

    private:
        int socket_fd_;
        rclcpp::Logger logger_;
};

#endif 

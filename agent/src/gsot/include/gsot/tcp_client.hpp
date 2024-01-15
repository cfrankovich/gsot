#ifndef TCP_CLIENT_HPP
#define TCP_CLIENT_HPP

#include <string>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <thread>
#include <stdexcept>
#include <cstring>
#include <netdb.h> 

class TCPClient {
    public:
        TCPClient(const std::string ip, uint16_t port, rclcpp::Logger logger);
        void sendData(const char *buffer, size_t buffer_size);
        void receiveData(std::function<void(const std::string &)> callback);
        ~TCPClient();

    private:
        int socket_fd_;
        rclcpp::Logger logger_;
        const std::string ip_; 
        uint16_t port_;

        void closeConnection();
        void establishConnection();
};

#endif 

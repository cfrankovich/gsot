#include "gsot/tcp_client.hpp"
#include <stdexcept>
#include <cstring>
#include <netdb.h> 
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <functional>
#include <chrono>
#include <thread>

TCPClient::TCPClient(const std::string ip, uint16_t port, rclcpp::Logger logger) : logger_(logger), ip_(ip), port_(port) {
    establishConnection();
}

void TCPClient::sendData(const char *buffer, size_t buffer_size) {
    ssize_t bytes_sent = send(socket_fd_, buffer, buffer_size, MSG_NOSIGNAL);
    if (bytes_sent < 0) {
        bool connected = false;
        while (!connected) {
            closeConnection();
            try {
                establishConnection();
                connected = true;
            } catch (const std::runtime_error& e) {
                RCLCPP_ERROR(logger_, "Reconnection failed | %s | Trying again in 5 seconds...", e.what());
                std::this_thread::sleep_for(std::chrono::seconds(5));
            }
        }
    }
}

void TCPClient::receiveData(std::function<void(const std::string &)> callback) {
    while (true) {
        char buffer[1024] = {0};
        int bytes_read = read(socket_fd_, buffer, 1024);
        if (bytes_read > 0) {
            std::string message(buffer, bytes_read);
            RCLCPP_INFO(logger_, "Data reads %s", message.c_str());
            callback(message);
        } else {
            closeConnection();
            try {
                establishConnection();
            } catch (const std::runtime_error& e) {
                RCLCPP_ERROR(logger_, "Reconnection failed | %s | Trying again in 5 seconds...", e.what());
                std::this_thread::sleep_for(std::chrono::seconds(5));
            }
        }
    }
}

void TCPClient::establishConnection() {
    if (socket_fd_ != -1) {
        closeConnection();
    }

    socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd_ == -1) {
        throw std::runtime_error("Failed to create socket");
    }

    int yes = 1;
    if (setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) == -1) {
        closeConnection();
        throw std::runtime_error("setsockopt SO_REUSEADDR failed");
    }

    struct addrinfo hints, *res;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET; // AF_INET means IPv4 only addresses
    hints.ai_socktype = SOCK_STREAM;

    const char* hostname = ip_.c_str(); 

    // Resolve the domain name into a list of addresses
    int result = getaddrinfo(hostname, std::to_string(port_).c_str(), &hints, &res);
    if (result != 0) {
        closeConnection();
        throw std::runtime_error("getaddrinfo: " + std::string(gai_strerror(result)));
    }

    // Connect to the first address in the list
    if (connect(socket_fd_, res->ai_addr, res->ai_addrlen) < 0) {
        freeaddrinfo(res); // Free the memory allocated by getaddrinfo
        closeConnection();
        throw std::runtime_error("Connection Failed");
    }

    freeaddrinfo(res); // Free the memory allocated by getaddrinfo
}

void TCPClient::closeConnection() {
    if (socket_fd_ != -1) {
        close(socket_fd_);
        socket_fd_ = -1;
    }
}

TCPClient::~TCPClient() {
    closeConnection();
}

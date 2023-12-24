#include "gsot/tcp_client.hpp"
#include <stdexcept>
#include <cstring>
#include <netdb.h> 

TCPClient::TCPClient(const std::string& ip, uint16_t port) {
    // Create socket
    socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd_ == -1) {
        throw std::runtime_error("Failed to create socket");
    }

    struct addrinfo hints, *res;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET; // AF_INET means IPv4 only addresses
    hints.ai_socktype = SOCK_STREAM;

    const char* hostname = ip.c_str(); 

    // Resolve the domain name into a list of addresses
    int result = getaddrinfo(hostname, std::to_string(port).c_str(), &hints, &res);
    if (result != 0) {
        throw std::runtime_error("getaddrinfo: " + std::string(gai_strerror(result)));
    }

    // Connect to the first address in the list
    if (connect(socket_fd_, res->ai_addr, res->ai_addrlen) < 0) {
        freeaddrinfo(res); // Free the memory allocated by getaddrinfo
        throw std::runtime_error("Connection Failed");
    }

    freeaddrinfo(res); // Free the memory allocated by getaddrinfo
}

void TCPClient::sendData(const char *buffer, size_t buffer_size) const {
    send(socket_fd_, buffer, buffer_size, 0);
}

void TCPClient::receiveData(std::function<void(const std::string &)> callback) const {
    char buffer[1024] = {0};
    int bytes_read = read(socket_fd_, buffer, 1024);
    if (bytes_read > 0) {
        std::string message(buffer, bytes_read);
        callback(message);
    }
}

TCPClient::~TCPClient() {
    if (socket_fd_ != -1) {
        close(socket_fd_);
    }
}

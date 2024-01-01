#include "gsot/udp_client.hpp"
#include <stdexcept>
#include <cstring>
#include <netdb.h> 
#include <iostream>

UDPClient::UDPClient(const std::string& ip, uint16_t port) {
    socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd == -1) {
        throw std::runtime_error("Failed to create socket");
    }

    const char* hostname = ip.c_str(); 

    struct addrinfo hints, *res;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;

    int result = getaddrinfo(hostname, std::to_string(port).c_str(), &hints, &res);

    if (result != 0) {
        throw std::runtime_error("getaddrinfo: " + std::string(gai_strerror(result)));
    }

    memcpy(&server_address, res->ai_addr, sizeof(struct sockaddr_in));

    freeaddrinfo(res);
}

void UDPClient::sendVideoData(const char *buffer, size_t buffer_size) const {
    std::cout << "HELLOOOOOO????\n";
    ssize_t bytes_sent = sendto(socket_fd, buffer, buffer_size, 0, (const struct sockaddr *) &server_address, sizeof(server_address));
    if (bytes_sent == -1) {
        std::cout << "bytes NOT sent\n";
    } else {
        std::cout << "bytes sent?\n";
    }
}

UDPClient::~UDPClient() {
    if (socket_fd != -1) {
        close(socket_fd);
    }
}

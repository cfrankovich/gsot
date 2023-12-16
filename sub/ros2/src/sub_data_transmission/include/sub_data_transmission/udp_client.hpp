#ifndef TRANSMISSION_CLIENT_HPP
#define TRANSMISSION_CLIENT_HPP

#include <string>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>

#define MAX_UDP_PACKET_SIZE 65527 // 65535 - 8 bytes 

class UDPClient {
    public:
        UDPClient(const std::string& ip, uint16_t port);
        void sendVideoData(const char *buffer, size_t buffer_size) const;
        ~UDPClient();

    private:
        int socket_fd;
        struct sockaddr_in server_address;
};

#endif 

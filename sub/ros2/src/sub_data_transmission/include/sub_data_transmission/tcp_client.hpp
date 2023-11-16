#ifndef TCP_CLIENT_HPP
#define TCP_CLIENT_HPP

#include <string>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>

class TCPClient {
    public:
        TCPClient(const std::string& ip, uint16_t port);
        void sendData(const std::string& data) const;
        ~TCPClient();

    private:
        int socket_fd_;
};

#endif 

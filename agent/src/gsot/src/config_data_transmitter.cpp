#include <thread>
#include <cstring>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "gsot/tcp_client.hpp"

class ConfigDataTransmitter: public rclcpp::Node 
{
    public:
        ConfigDataTransmitter()
        : Node("config_data_transmitter"),
        tcp_client("receiver", 12303, get_logger())
        {
            std::thread([this]() {
                tcp_client.receiveData([this](const std::string &message) {
                    if (message == "TRANSMIT_ALL_TOPICS") {
                        this->transmitAllTopics();
                    }
                });
            }).detach();
        }

    private:
        void transmitAllTopics()
        {
            auto topics_and_types = this->get_topic_names_and_types();
            std::stringstream topics_stream;

            for (const auto& info : topics_and_types)
            {
                topics_stream << info.first << "\n";
            }

            std::string topics_str = topics_stream.str();
            const char* topics_c_str = topics_str.c_str(); 
            size_t topics_c_str_size = strlen(topics_c_str);
            tcp_client.sendData(topics_c_str, topics_c_str_size);
        }

        TCPClient tcp_client;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ConfigDataTransmitter>());
    rclcpp::shutdown();
    return 0;
}
#include "rclcpp/rclcpp.hpp"
#include "gsot/tcp_client.hpp"
#include "std_msgs/msg/string.hpp"

class MetricDataTransmitter: public rclcpp::Node 
{
    public:
        MetricDataTransmitter() 
        : Node("metric_data_transmitter"), 
        tcp_client("receiver", 12301, get_logger())
        {
            subscription = this->create_subscription<std_msgs::msg::String>(
                "gsot/aggregated_data",
                10,
                std::bind(&MetricDataTransmitter::callback, this, std::placeholders::_1)
            );
        }

    private:
        TCPClient tcp_client;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription;
        rclcpp::Node::SharedPtr node_handle;

        void callback(const std_msgs::msg::String::SharedPtr msg)
        {
            const char *data = msg->data.c_str();
            size_t size = strlen(data);
            tcp_client.sendData(data, size);
        }
};

int main(int argc, char* argv[]) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MetricDataTransmitter>());
    rclcpp::shutdown();
    return 0;
}

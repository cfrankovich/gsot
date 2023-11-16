#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sub_data_transmission/tcp_client.hpp"

#include "sub_pub_tester/msg/battery_levels.hpp"

class MetricDataTransmitter: public rclcpp::Node 
{
    public:
        MetricDataTransmitter() 
        : Node("metric_data_transmitter"), 
        tcp_client("receiver", 12345)
        {
            subscription = this->create_subscription<sub_pub_tester::msg::BatteryLevels>(
                "battery_levels", 
                10,
                std::bind(&MetricDataTransmitter::battery_levels_callback, this, std::placeholders::_1)
            );
        }

    private:
        void battery_levels_callback(const sub_pub_tester::msg::BatteryLevels::SharedPtr msg) const 
        {
            RCLCPP_INFO_STREAM(
                this->get_logger(), 
                "Battery Levels: '" << msg->battery0 << " " << msg->battery1 << " " << msg->battery2 << " " << msg->battery3 << "'"
            );

            // Assuming you want to send the concatenated battery levels as a string
            const std::string data = std::to_string(msg->battery0) + " " +
                            std::to_string(msg->battery1) + " " +
                            std::to_string(msg->battery2) + " " +
                            std::to_string(msg->battery3);
            tcp_client.sendData(data);
        }
        TCPClient tcp_client;
        rclcpp::Subscription<sub_pub_tester::msg::BatteryLevels>::SharedPtr subscription;
};

int main(int argc, char *argv[]) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MetricDataTransmitter>());
    rclcpp::shutdown();
    return 0;
}

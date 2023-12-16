#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sub_data_transmission/tcp_client.hpp"
#include "sub_pub_tester/msg/battery_levels.hpp"
#include "sub_pub_tester/msg/orientation.hpp"

struct MetricData 
{
    // Battery Levels
    double battery0;
    double battery1;
    double battery2;
    double battery3;

    // Orientation
    double pitch;
    double roll;
    double yaw;

    // Speed
    double speed;

    // Time
    time_t current_time; 
}; 

class MetricDataTransmitter: public rclcpp::Node 
{
    public:
        MetricDataTransmitter() 
        : Node("metric_data_transmitter"), 
        metric_data(),
        tcp_client("receiver", 12301)
        {
            battery_levels_subscription = this->create_subscription<sub_pub_tester::msg::BatteryLevels>(
                "battery_levels", 
                10,
                std::bind(&MetricDataTransmitter::battery_levels_callback, this, std::placeholders::_1)
            );

            orientation_subscription = this->create_subscription<sub_pub_tester::msg::Orientation>(
                "orientation",
                10,
                std::bind(&MetricDataTransmitter::orientation_callback, this, std::placeholders::_1)
            );

            speed_subscription = this->create_subscription<std_msgs::msg::Float64>(
                "speed",
                10,
                std::bind(&MetricDataTransmitter::speed_callback, this, std::placeholders::_1)
            );

            sender_thread = std::thread(&MetricDataTransmitter::transmitData, this);
        }

    private:
        void battery_levels_callback(const sub_pub_tester::msg::BatteryLevels::SharedPtr msg) const 
        {
            std::lock_guard<std::mutex> lock(data_mutex);
            metric_data.battery0 = msg->battery0;
            metric_data.battery1 = msg->battery1;
            metric_data.battery2 = msg->battery2;
            metric_data.battery3 = msg->battery3;
        }

        void orientation_callback(const sub_pub_tester::msg::Orientation::SharedPtr msg) const
        {
            std::lock_guard<std::mutex> lock(data_mutex);
            metric_data.pitch = msg->pitch;
            metric_data.roll= msg->roll;
            metric_data.yaw = msg->yaw;
        }

        void speed_callback(const std_msgs::msg::Float64::SharedPtr msg) const
        {
            std::lock_guard<std::mutex> lock(data_mutex);
            metric_data.speed = msg->data;
        }

        rclcpp::Subscription<sub_pub_tester::msg::BatteryLevels>::SharedPtr battery_levels_subscription;
        rclcpp::Subscription<sub_pub_tester::msg::Orientation>::SharedPtr orientation_subscription;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_subscription; 

        mutable MetricData metric_data;

        std::thread sender_thread;
        mutable std::mutex data_mutex;
        bool running = true;
        const unsigned int LATENCY = 1000; // the interval between each transmission (ms)

        void transmitData()
        {
            while (running) {
                std::this_thread::sleep_for(std::chrono::milliseconds(LATENCY));
                sendData();
            }
        }

        void sendData()
        {
            std::lock_guard<std::mutex> lock(data_mutex);

            metric_data.current_time = time(nullptr);

            const char* buffer = reinterpret_cast<const char*>(&metric_data);
            size_t buffer_size = sizeof(metric_data);

            tcp_client.sendData(buffer, buffer_size);
        }

        TCPClient tcp_client;
};

int main(int argc, char* argv[]) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MetricDataTransmitter>());
    rclcpp::shutdown();
    return 0;
}

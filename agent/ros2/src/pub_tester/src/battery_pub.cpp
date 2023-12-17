#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "pub_tester/msg/battery_levels.hpp"

class BatteryPublisher: public rclcpp::Node 
{
    public:
        BatteryPublisher() : Node("battery_levels_publisher") 
        {
            publisher = this->create_publisher<pub_tester::msg::BatteryLevels>("battery_levels", 10);
            timer = this->create_wall_timer(period, std::bind(&BatteryPublisher::timer_callback, this)); 
        }

    private:
        void timer_callback()
        {
            auto message = pub_tester::msg::BatteryLevels();
            message.battery0 = 100.0;
            message.battery1 = 75.0;
            message.battery2 = 50.0;
            message.battery3 = 25.0;
            publisher->publish(message);
        }
        static constexpr std::chrono::milliseconds period = std::chrono::milliseconds(500); 
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Publisher<pub_tester::msg::BatteryLevels>::SharedPtr publisher;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BatteryPublisher>());
    rclcpp::shutdown();
    return 0;
}

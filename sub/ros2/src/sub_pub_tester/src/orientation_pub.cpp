#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "sub_pub_tester/msg/orientation.hpp"

#include <cmath>

class OrientationPublisher: public rclcpp::Node 
{
    public:
        OrientationPublisher() : Node("orientation_publisher") 
        {
            publisher = this->create_publisher<sub_pub_tester::msg::Orientation>("orientation", 10);
            timer = this->create_wall_timer(period, std::bind(&OrientationPublisher::timer_callback, this)); 
        }

    private:
        void timer_callback()
        {
            auto message = sub_pub_tester::msg::Orientation();
            message.pitch = M_PI;
            message.roll = 2 * M_PI;
            message.yaw = 3 * M_PI;
            publisher->publish(message);
        }
        static constexpr std::chrono::milliseconds period = std::chrono::milliseconds(500); 
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Publisher<sub_pub_tester::msg::Orientation>::SharedPtr publisher;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OrientationPublisher>());
    rclcpp::shutdown();
    return 0;
}

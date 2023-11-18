#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class SpeedPublisher: public rclcpp::Node 
{
    public:
        SpeedPublisher() : Node("speed_publisher") 
        {
            publisher = this->create_publisher<std_msgs::msg::Float64>("speed", 10);
            timer = this->create_wall_timer(period, std::bind(&SpeedPublisher::timer_callback, this)); 
        }

    private:
        void timer_callback()
        {
            auto message = std_msgs::msg::Float64();
            message.data = 44.44;
            publisher->publish(message);
        }
        static constexpr std::chrono::milliseconds period = std::chrono::milliseconds(500); 
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpeedPublisher>());
    rclcpp::shutdown();
    return 0;
}

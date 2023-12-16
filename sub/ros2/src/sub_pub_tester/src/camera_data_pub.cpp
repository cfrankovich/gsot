#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp> 
#include "cv_bridge/cv_bridge.h"

class CameraPublisher: public rclcpp::Node
{
    public:
        CameraPublisher(): 
            Node("camera_data_publisher"),
            node_handle(std::shared_ptr<CameraPublisher>(this, [](auto *) {})),
            it(node_handle),
            publisher(it.advertise("video/frame", 1)),
            cap("/root/workspace/ros2/src/sub_pub_tester/src/sample-video.mp4")
        {

            timer = this->create_wall_timer(period, std::bind(&CameraPublisher::timer_callback, this)); 

            if (!cap.isOpened()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to read video.");
            }
        }

    private:
        void timer_callback()
        {
            cv::Mat frame;
            sensor_msgs::msg::Image::SharedPtr msg;
            std_msgs::msg::Header hdr;            

            cap >> frame; // "The >> operator is overloaded in OpenCV to provide a convenient way to read the next frame"
            if (!frame.empty()) {
                msg = cv_bridge::CvImage(hdr, "bgr8", frame).toImageMsg();
                publisher.publish(msg);
                cv::waitKey(1);
            }
        }
        rclcpp::Node::SharedPtr node_handle;
        image_transport::ImageTransport it;
        image_transport::Publisher publisher;
        cv::VideoCapture cap;
        static constexpr std::chrono::milliseconds period = std::chrono::milliseconds(500); 
        rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraPublisher>());
    rclcpp::shutdown();
    return 0;
}
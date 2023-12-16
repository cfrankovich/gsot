#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "sub_data_transmission/udp_client.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp> 
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/image.hpp"

class CameraDataTransmitter: public rclcpp::Node 
{
    public:
        CameraDataTransmitter() : 
            Node("camera_data_transmitter"), 
            node_handle(std::shared_ptr<CameraDataTransmitter>(this, [](auto *) {})),
            it(node_handle),
            udp_client("receiver", 12302)
        {
            subscription = it.subscribe("video/frame", 1, std::bind(&CameraDataTransmitter::callback, this, std::placeholders::_1)); 
        }

    private:
        void callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) 
        {
            try {
                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
                cv::Mat frame = cv_ptr->image;

                if (!frame.empty()) {
                    std::vector<uchar> buffer;
                    cv::imencode(".jpg", frame, buffer);

                    size_t frame_size = buffer.size();
                    char size_buf[4];
                    size_buf[0] = (frame_size >> 24) & 0xFF;
                    size_buf[1] = (frame_size >> 16) & 0xFF;
                    size_buf[2] = (frame_size >> 8) & 0xFF;
                    size_buf[3] = frame_size & 0xFF;

                    //sendData(size_buf, 4);
                    sendData(reinterpret_cast<const char*>(buffer.data()), buffer.size());
                }
            } catch (cv_bridge::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            }
        }

        void sendData(const char* buffer, size_t buffer_size)
        {
            if (buffer_size > MAX_UDP_PACKET_SIZE) {
                // TODO: fragmentation 
            } else {
                RCLCPP_INFO(this->get_logger(), "sending data?"); 
                udp_client.sendVideoData(buffer, buffer_size);
            }
        }
        rclcpp::Node::SharedPtr node_handle;
        image_transport::ImageTransport it;
        image_transport::Subscriber subscription;
        UDPClient udp_client;
};

int main(int argc, char* argv[]) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraDataTransmitter>());
    rclcpp::shutdown();
    return 0;
}

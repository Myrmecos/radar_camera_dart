#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"
#include "hikrobot_camera.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class ImagePublisher : public rclcpp::Node
{
  public:
    ImagePublisher(): 
    Node("image_publisher"),
    camera_left(std::make_unique<camera::Camera>(0))
    {
        
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/hikrobot_image", 10);
        timer_=this->create_wall_timer(
            std::chrono::milliseconds(33),
            [this]() { this->publishLatestImage(); }
        );
    }

    void publishLatestImage() {
        if (!camera_left) {
            RCLCPP_ERROR(this->get_logger(), "camera_left is nullptr!");
            return;
        }
        std::vector<cv::Mat> src(2);
        // step 1. obtain image
        auto leftFuture = camera_left->asyncGetImage(src[0]);
        leftFuture.get();
        cv::Mat left_image = src[0];

        // step 2. check if left image is empty
        if (left_image.empty()) {
            RCLCPP_WARN(this->get_logger(), "received empty image.");
            return;
        }

        // step 3. generate image and publish
        auto msg = cv_bridge::CvImage(
            std_msgs::msg::Header(), 
            "bgr8",  // Assuming 3-channel BGR
            left_image
        ).toImageMsg();

        msg->header.frame_id = "left_camera";
        msg->header.stamp = this->now();
        publisher_->publish(*msg);

        RCLCPP_DEBUG(this->get_logger(), "Published image!");


    }

  private:
    
    std::unique_ptr<camera::Camera> camera_left;
    std::unique_ptr<camera::Camera> camera_right;
    int frame_rate = 0, frame_count = 0;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImagePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#include <atomic>
#include <chrono>
#include <functional>
#include <future>
#include <iostream>
#include "opencv2/opencv.hpp"
#include "hikrobot_camera.hpp"
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include <string>

class HikrobotCameraNode : public rclcpp::Node
{
public:
    HikrobotCameraNode()
        : Node("hikrobot_camera_node"),
          camera_left(std::make_unique<camera::Camera>(0)),
          camera_right(nullptr)
    {
        rclcpp::Logger logger = rclcpp::get_logger("hikrobot_camera_node");
        logger.set_level(rclcpp::Logger::Level::Info);
    }

    void captureImages()
    {
        std::vector<cv::Mat> src(2);

        std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();
        RCLCPP_ERROR(this->get_logger(), "fuck1");
        int timestamp = 0;
        while (rclcpp::ok())
        {
            auto leftFuture = camera_left->asyncGetImage(src[0]);
            // auto rightFuture = camera_right->asyncGetImage(src[1]);
            leftFuture.get();
            
            cv::imshow("left image", src[0]);
            std::stringstream ss;
            ss<<timestamp;
            //std::string filename = "/home/astar/data/green_light/" + ss.str() + ".jpg";
            timestamp++;
            // cv::waitKey(10);
            //cv::imwrite(filename, src[0]);
            cv::waitKey(500);
            // rightkFuture.get();
            
            frame_count++;
            std::chrono::high_resolution_clock::time_point end_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> elapsed_time = end_time - start_time;
            if (elapsed_time.count() >= 1000)
            {
                frame_rate = frame_count;
                frame_count = 0;
                start_time = end_time;
                RCLCPP_INFO(this->get_logger(), "Frame Rate: %d fps", frame_rate);
            } 
        }
    }

private:
    std::unique_ptr<camera::Camera> camera_left;
    std::unique_ptr<camera::Camera> camera_right;
    int frame_rate = 0, frame_count = 0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HikrobotCameraNode>();

    node->captureImages();

    rclcpp::shutdown();
    return 0;
}
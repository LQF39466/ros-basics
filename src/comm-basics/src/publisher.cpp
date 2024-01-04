#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

class TextVideoPublisher : public rclcpp::Node
{
public:
  TextVideoPublisher()
    : Node("text_video_publisher"), count_(0)
  {
    text_publisher_ = this->create_publisher<std_msgs::msg::String>("text_topic", 10);
    video_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("video_topic", 10);

    text_timer_ = this->create_wall_timer(
      500ms, std::bind(&TextVideoPublisher::text_timer_callback, this));
    video_timer_ = this->create_wall_timer(
      33ms, std::bind(&TextVideoPublisher::video_timer_callback, this));
  }

private:
  void text_timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing text: '%s'", message.data.c_str());
    text_publisher_->publish(message);
  }

  void video_timer_callback()
  {
    cv::VideoCapture cap("video/simple.mp4"); // Replace with the path to your video file

    if (!cap.isOpened())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open video file");
      return;
    }

    cv::Mat frame;
    cap >> frame;

    if (frame.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to read video frame");
      return;
    }

    std::unique_ptr<sensor_msgs::msg::Image> image_msg = std::make_unique<sensor_msgs::msg::Image>();
    image_msg->header.stamp = this->get_clock()->now();
    image_msg->header.frame_id = "camera_frame";
    image_msg->height = frame.rows;
    image_msg->width = frame.cols;
    image_msg->encoding = "bgr8";
    image_msg->step = frame.cols * frame.elemSize();
    size_t size = frame.total() * frame.elemSize();
    image_msg->data.resize(size);
    memcpy(&image_msg->data[0], frame.data, size);

    RCLCPP_INFO(this->get_logger(), "Publishing video frame");
    video_publisher_->publish(std::move(image_msg));
  }

  rclcpp::TimerBase::SharedPtr text_timer_;
  rclcpp::TimerBase::SharedPtr video_timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr text_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr video_publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TextVideoPublisher>());
  rclcpp::shutdown();
  return 0;
}

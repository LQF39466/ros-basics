#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <opencv2/opencv.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

class TextVideoSubscriber : public rclcpp::Node
{
public:
  TextVideoSubscriber()
    : Node("text_video_subscriber")
  {
    text_subscription_ = this->create_subscription<std_msgs::msg::String>(
      "text_topic", 10, std::bind(&TextVideoSubscriber::text_topic_callback, this, _1));
    video_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "video_topic", 10, std::bind(&TextVideoSubscriber::video_topic_callback, this, _1));
    
    cv::namedWindow("Video", cv::WINDOW_NORMAL); // Create a named window for video display
    cv::startWindowThread(); // Start the thread for handling GUI events
  }

private:
  void text_topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received text: '%s'", msg->data.c_str());
  }

  void video_topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
  {
    cv::Mat frame(msg->height, msg->width, CV_8UC3, const_cast<unsigned char*>(msg->data.data()), msg->step);

    if (frame.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to create video frame");
      return;
    }

    cv::imshow("Video", frame);
    cv::waitKey(1);
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr text_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr video_subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto subscriber = std::make_shared<TextVideoSubscriber>();
  rclcpp::spin(subscriber);
  rclcpp::shutdown();
  return 0;
}

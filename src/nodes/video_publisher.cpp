// src/nodes/video_publisher.cpp
#include <opencv2/videoio.hpp>
#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"

class DualVideoPublisher : public rclcpp::Node {
public:
  DualVideoPublisher() : Node("dual_video_publisher") {
    // 声明参数
    declare_parameter("left_source", "left.mp4");
    declare_parameter("right_source", "right.mp4");
    declare_parameter("frame_rate", 30.0);
    
    // 获取参数
    const auto left_path = get_parameter("left_source").as_string();
    const auto right_path = get_parameter("right_source").as_string();
    const double fps = get_parameter("frame_rate").as_double();

    // 初始化视频源
    if(!left_cap_.open(left_path) || !right_cap_.open(right_path)) {
      RCLCPP_ERROR(get_logger(), "Failed to open video files!");
      rclcpp::shutdown();
    }

    // 创建发布器
    left_pub_ = image_transport::create_publisher(this, "camera/left");
    right_pub_ = image_transport::create_publisher(this, "camera/right");

    // 定时器发布
    const auto interval = std::chrono::milliseconds(static_cast<int>(1000/fps));
    timer_ = create_wall_timer(interval, [this](){ publish_frames(); });
  }

private:
  void publish_frames() {
    auto left_frame = capture_frame(left_cap_, "left_camera");
    auto right_frame = capture_frame(right_cap_, "right_camera");
    
    if(left_frame && right_frame) {
      left_pub_.publish(left_frame);
      right_pub_.publish(right_frame);
    }
  }

  sensor_msgs::msg::Image::SharedPtr capture_frame(cv::VideoCapture& cap, const std::string& frame_id) {
    cv::Mat frame;
    if(cap.read(frame)) {
      auto msg = cv_bridge::CvImage(
        std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
      msg->header.stamp = now();
      msg->header.frame_id = frame_id;
      return msg;
    }
    return nullptr;
  }

  cv::VideoCapture left_cap_, right_cap_;
  image_transport::Publisher left_pub_, right_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DualVideoPublisher>());
  rclcpp::shutdown();
  return 0;
}
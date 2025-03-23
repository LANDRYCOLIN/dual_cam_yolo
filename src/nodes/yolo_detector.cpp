// src/nodes/yolo_detector.cpp
#include <opencv2/dnn.hpp>
#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "vision_msgs/vision_msgs/msg/bounding_box2_d_array.hpp"

class YoloDetector : public rclcpp::Node {
public:
  YoloDetector() : Node("yolo_detector") {
    // 加载参数
    declare_parameter("model_path", "yolov8n.onnx");
    declare_parameter("confidence_threshold", 0.5);
    declare_parameter("input_size", 640);
    
    // 初始化模型
    const std::string model_path = get_parameter("model_path").as_string();
    net_ = cv::dnn::readNet(model_path);
    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

    // 创建订阅器和发布器
    sub_ = image_transport::create_subscription(
      this, "camera/left",
      [this](const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        process_frame(msg);
      }, "raw");
    
    detections_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>(
      "detections", 10);
  }

private:
  void process_frame(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg) {
    // 转换为OpenCV格式
    cv::Mat frame = cv_bridge::toCvCopy(img_msg, "bgr8")->image;

    // 预处理
    cv::Mat blob;
    cv::dnn::blobFromImage(frame, blob, 1/255.0, cv::Size(640,640));
    
    // 推理
    net_.setInput(blob);
    cv::Mat outputs = net_.forward();

    // 解析结果
    vision_msgs::msg::Detection2DArray detections;
    detections.header = img_msg->header;
    
    const float* data = (float*)outputs.data;
    for(int i = 0; i < outputs.rows; ++i) {
      const float confidence = data[4];
      if(confidence > confidence_threshold_) {
        vision_msgs::msg::Detection2D detection;
        detection.bbox.center.position.x = data[0] * frame.cols;
        detection.bbox.center.position.y = data[1] * frame.rows;
        detection.bbox.size_x = data[2] * frame.cols;
        detection.bbox.size_y = data[3] * frame.rows;
        detection.results.resize(1);
        detection.results[0].hypothesis.class_id = static_cast<int>(data[5]);
        detection.results[0].hypothesis.score = confidence;
        detections.detections.push_back(detection);
      }
      data += outputs.cols;
    }

    detections_pub_->publish(detections);
  }

  cv::dnn::Net net_;
  image_transport::Subscriber sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detections_pub_;
  float confidence_threshold_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<YoloDetector>());
  rclcpp::shutdown();
  return 0;
}
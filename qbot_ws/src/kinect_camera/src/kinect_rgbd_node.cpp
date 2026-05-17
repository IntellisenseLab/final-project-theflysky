// Publishes Kinect 360 RGB + depth straight from the libfreenect callbacks
// instead of polling a buffer from a wall timer. That removes the timer-vs-
// camera rate mismatch (which both drops fresh frames and republishes stale
// ones at the same time) and shaves a frame-worth of latency off the gesture
// pipeline.

#include <atomic>
#include <chrono>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "libfreenect.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

class KinectRgbdNode : public rclcpp::Node
{
public:
  KinectRgbdNode()
  : Node("kinect_rgbd_node")
  {
    device_index_ = declare_parameter<int>("device_index", 0);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 30.0);
    enable_rgb_ = declare_parameter<bool>("enable_rgb", true);
    enable_depth_ = declare_parameter<bool>("enable_depth", true);
    rgb_topic_name_ = declare_parameter<std::string>("rgb_topic", "/kinect/rgb/image_raw");
    depth_topic_name_ = declare_parameter<std::string>("depth_topic", "/kinect/depth/image_raw");
    rgb_frame_id_ = declare_parameter<std::string>("rgb_frame_id", "kinect_rgb_optical_frame");
    depth_frame_id_ = declare_parameter<std::string>("depth_frame_id", "kinect_depth_optical_frame");

    if (publish_rate_hz_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "publish_rate_hz must be > 0. Falling back to 30 Hz.");
      publish_rate_hz_ = 30.0;
    }
    min_publish_interval_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz_));

    if (enable_rgb_) {
      rgb_publisher_ = create_publisher<sensor_msgs::msg::Image>(
        rgb_topic_name_, rclcpp::SensorDataQoS());
    }
    if (enable_depth_) {
      depth_publisher_ = create_publisher<sensor_msgs::msg::Image>(
        depth_topic_name_, rclcpp::SensorDataQoS());
    }

    if (!enable_rgb_ && !enable_depth_) {
      RCLCPP_WARN(
        get_logger(),
        "Both enable_rgb and enable_depth are false. The node will stay idle until one stream is enabled.");
    } else if (!initialize_device()) {
      RCLCPP_ERROR(
        get_logger(),
        "Kinect initialization failed. The node will stay alive so you can inspect logs.");
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "Kinect node ready. RGB=%s (%s), Depth=%s (%s), device_index=%d, max_rate=%.1f Hz",
      enable_rgb_ ? "on" : "off",
      rgb_topic_name_.c_str(),
      enable_depth_ ? "on" : "off",
      depth_topic_name_.c_str(),
      device_index_,
      publish_rate_hz_);
  }

  ~KinectRgbdNode() override
  {
    shutdown_device();
  }

private:
  static constexpr std::size_t kRgbWidth = 640;
  static constexpr std::size_t kRgbHeight = 480;
  static constexpr std::size_t kRgbBytesPerPixel = 3;
  static constexpr std::size_t kDepthBytesPerPixel = sizeof(uint16_t);

  static void video_callback(freenect_device * dev, void * video, uint32_t)
  {
    auto * self = static_cast<KinectRgbdNode *>(freenect_get_user(dev));
    if (self != nullptr) {
      self->publish_rgb_frame(video);
    }
  }

  static void depth_callback(freenect_device * dev, void * depth, uint32_t)
  {
    auto * self = static_cast<KinectRgbdNode *>(freenect_get_user(dev));
    if (self != nullptr) {
      self->publish_depth_frame(depth);
    }
  }

  bool initialize_device()
  {
    if (freenect_init(&context_, nullptr) < 0) {
      RCLCPP_ERROR(get_logger(), "freenect_init failed.");
      return false;
    }

    freenect_set_log_level(context_, FREENECT_LOG_ERROR);
    freenect_select_subdevices(context_, FREENECT_DEVICE_CAMERA);

    const int device_count = freenect_num_devices(context_);
    if (device_count <= device_index_) {
      RCLCPP_ERROR(
        get_logger(),
        "Requested Kinect index %d, but only %d device(s) were found.",
        device_index_, device_count);
      shutdown_device();
      return false;
    }

    if (freenect_open_device(context_, &device_, device_index_) < 0) {
      RCLCPP_ERROR(
        get_logger(),
        "freenect_open_device failed for index %d. Check USB access and firmware.",
        device_index_);
      shutdown_device();
      return false;
    }

    freenect_set_user(device_, this);

    if (enable_rgb_) {
      const auto video_mode = freenect_find_video_mode(
        FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB);
      if (!video_mode.is_valid || freenect_set_video_mode(device_, video_mode) < 0) {
        RCLCPP_ERROR(get_logger(), "Failed to configure Kinect RGB mode.");
        shutdown_device();
        return false;
      }
      rgb_buffer_.resize(video_mode.bytes);
      freenect_set_video_callback(device_, &KinectRgbdNode::video_callback);
    }

    if (enable_depth_) {
      const auto depth_mode = freenect_find_depth_mode(
        FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_MM);
      if (!depth_mode.is_valid || freenect_set_depth_mode(device_, depth_mode) < 0) {
        RCLCPP_ERROR(get_logger(), "Failed to configure Kinect depth mode.");
        shutdown_device();
        return false;
      }
      depth_buffer_.resize(depth_mode.bytes);
      freenect_set_depth_callback(device_, &KinectRgbdNode::depth_callback);
    }

    if (enable_depth_ && freenect_start_depth(device_) < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to start Kinect depth stream.");
      shutdown_device();
      return false;
    }
    if (enable_rgb_ && freenect_start_video(device_) < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to start Kinect RGB stream.");
      shutdown_device();
      return false;
    }

    running_ = true;
    event_thread_ = std::thread(&KinectRgbdNode::event_loop, this);
    return true;
  }

  void shutdown_device()
  {
    running_ = false;
    if (event_thread_.joinable()) {
      event_thread_.join();
    }
    if (device_ != nullptr) {
      if (enable_rgb_) {
        freenect_stop_video(device_);
      }
      if (enable_depth_) {
        freenect_stop_depth(device_);
      }
      freenect_close_device(device_);
      device_ = nullptr;
    }
    if (context_ != nullptr) {
      freenect_shutdown(context_);
      context_ = nullptr;
    }
  }

  void event_loop()
  {
    while (running_ && context_ != nullptr) {
      timeval timeout{};
      timeout.tv_sec = 0;
      timeout.tv_usec = 50000;
      const int status = freenect_process_events_timeout(context_, &timeout);
      if (status < 0 && running_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "libfreenect event processing reported an error. Check Kinect USB power and permissions.");
      }
    }
  }

  bool rate_limited(std::chrono::steady_clock::time_point & last_publish)
  {
    const auto now = std::chrono::steady_clock::now();
    if (now - last_publish < min_publish_interval_) {
      return true;
    }
    last_publish = now;
    return false;
  }

  void publish_rgb_frame(void * frame)
  {
    if (!enable_rgb_ || rgb_publisher_ == nullptr || frame == nullptr) {
      return;
    }
    if (rate_limited(rgb_last_publish_)) {
      return;
    }

    std::vector<uint8_t> copy;
    {
      std::lock_guard<std::mutex> lock(rgb_mutex_);
      std::memcpy(rgb_buffer_.data(), frame, rgb_buffer_.size());
      copy = rgb_buffer_;
    }

    sensor_msgs::msg::Image image_msg;
    image_msg.header.stamp = now();
    image_msg.header.frame_id = rgb_frame_id_;
    image_msg.height = kRgbHeight;
    image_msg.width = kRgbWidth;
    image_msg.encoding = sensor_msgs::image_encodings::RGB8;
    image_msg.is_bigendian = false;
    image_msg.step = kRgbWidth * kRgbBytesPerPixel;
    image_msg.data = std::move(copy);
    rgb_publisher_->publish(std::move(image_msg));
  }

  void publish_depth_frame(void * frame)
  {
    if (!enable_depth_ || depth_publisher_ == nullptr || frame == nullptr) {
      return;
    }
    if (rate_limited(depth_last_publish_)) {
      return;
    }

    std::vector<uint8_t> copy;
    {
      std::lock_guard<std::mutex> lock(depth_mutex_);
      std::memcpy(depth_buffer_.data(), frame, depth_buffer_.size());
      copy = depth_buffer_;
    }

    sensor_msgs::msg::Image image_msg;
    image_msg.header.stamp = now();
    image_msg.header.frame_id = depth_frame_id_;
    image_msg.height = kRgbHeight;
    image_msg.width = kRgbWidth;
    image_msg.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    image_msg.is_bigendian = false;
    image_msg.step = kRgbWidth * kDepthBytesPerPixel;
    image_msg.data = std::move(copy);
    depth_publisher_->publish(std::move(image_msg));
  }

  int device_index_{};
  double publish_rate_hz_{};
  std::chrono::nanoseconds min_publish_interval_{};
  bool enable_rgb_{};
  bool enable_depth_{};
  std::string rgb_topic_name_;
  std::string depth_topic_name_;
  std::string rgb_frame_id_;
  std::string depth_frame_id_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_publisher_;
  freenect_context * context_{nullptr};
  freenect_device * device_{nullptr};
  std::atomic<bool> running_{false};
  std::thread event_thread_;
  std::mutex rgb_mutex_;
  std::mutex depth_mutex_;
  std::vector<uint8_t> rgb_buffer_;
  std::vector<uint8_t> depth_buffer_;
  std::chrono::steady_clock::time_point rgb_last_publish_{};
  std::chrono::steady_clock::time_point depth_last_publish_{};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KinectRgbdNode>());
  rclcpp::shutdown();
  return 0;
}

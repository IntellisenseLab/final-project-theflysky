// Publishes Kinect 360 RGB + depth from libfreenect. The libfreenect USB
// callback only stashes the newest frame (a single memcpy); a separate
// publisher thread does the heavy ROS/DDS publish. Doing the ~900 KB DDS
// publish directly inside the USB callback used to block libfreenect's event
// loop long enough to starve the isochronous stream, which capped the camera
// at ~9 fps. Decoupling restores the full ~30 fps the hardware delivers.

#include <atomic>
#include <chrono>
#include <condition_variable>
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
    // 10% margin so that a target rate equal to the camera's native rate
    // (e.g. 30 Hz) is not throttled down by frame-interval jitter, while a
    // genuinely lower target still caps the publish rate as intended.
    min_publish_interval_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(0.9 / publish_rate_hz_));

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
      self->stash_rgb_frame(video);
    }
  }

  static void depth_callback(freenect_device * dev, void * depth, uint32_t)
  {
    auto * self = static_cast<KinectRgbdNode *>(freenect_get_user(dev));
    if (self != nullptr) {
      self->stash_depth_frame(depth);
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
    publish_thread_ = std::thread(&KinectRgbdNode::publish_loop, this);
    event_thread_ = std::thread(&KinectRgbdNode::event_loop, this);
    return true;
  }

  void shutdown_device()
  {
    running_ = false;
    frame_cv_.notify_all();
    if (publish_thread_.joinable()) {
      publish_thread_.join();
    }
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

  // Runs on the libfreenect event thread: copy the newest frame and return
  // immediately so USB event processing is never blocked by a DDS publish.
  void stash_rgb_frame(void * frame)
  {
    if (!enable_rgb_ || frame == nullptr) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      std::memcpy(rgb_buffer_.data(), frame, rgb_buffer_.size());
      rgb_ready_ = true;
    }
    frame_cv_.notify_one();
  }

  void stash_depth_frame(void * frame)
  {
    if (!enable_depth_ || frame == nullptr) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      std::memcpy(depth_buffer_.data(), frame, depth_buffer_.size());
      depth_ready_ = true;
    }
    frame_cv_.notify_one();
  }

  // Dedicated thread: waits for a stashed frame, then does the heavy publish.
  void publish_loop()
  {
    while (running_) {
      std::vector<uint8_t> rgb_copy;
      std::vector<uint8_t> depth_copy;
      bool have_rgb = false;
      bool have_depth = false;
      {
        std::unique_lock<std::mutex> lock(frame_mutex_);
        frame_cv_.wait_for(lock, std::chrono::milliseconds(100), [this] {
          return rgb_ready_ || depth_ready_ || !running_;
        });
        if (!running_) {
          break;
        }
        if (rgb_ready_) {
          rgb_copy = rgb_buffer_;
          rgb_ready_ = false;
          have_rgb = true;
        }
        if (depth_ready_) {
          depth_copy = depth_buffer_;
          depth_ready_ = false;
          have_depth = true;
        }
      }
      if (have_rgb && rgb_publisher_ != nullptr && !rate_limited(rgb_last_publish_)) {
        publish_rgb_frame(std::move(rgb_copy));
      }
      if (have_depth && depth_publisher_ != nullptr && !rate_limited(depth_last_publish_)) {
        publish_depth_frame(std::move(depth_copy));
      }
    }
  }

  void publish_rgb_frame(std::vector<uint8_t> frame)
  {
    sensor_msgs::msg::Image image_msg;
    image_msg.header.stamp = now();
    image_msg.header.frame_id = rgb_frame_id_;
    image_msg.height = kRgbHeight;
    image_msg.width = kRgbWidth;
    image_msg.encoding = sensor_msgs::image_encodings::RGB8;
    image_msg.is_bigendian = false;
    image_msg.step = kRgbWidth * kRgbBytesPerPixel;
    image_msg.data = std::move(frame);
    rgb_publisher_->publish(std::move(image_msg));
  }

  void publish_depth_frame(std::vector<uint8_t> frame)
  {
    sensor_msgs::msg::Image image_msg;
    image_msg.header.stamp = now();
    image_msg.header.frame_id = depth_frame_id_;
    image_msg.height = kRgbHeight;
    image_msg.width = kRgbWidth;
    image_msg.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    image_msg.is_bigendian = false;
    image_msg.step = kRgbWidth * kDepthBytesPerPixel;
    image_msg.data = std::move(frame);
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
  std::thread publish_thread_;
  std::mutex frame_mutex_;
  std::condition_variable frame_cv_;
  bool rgb_ready_{false};
  bool depth_ready_{false};
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

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <csignal>

using namespace cv;

// Definitions for Boson640
#define DEVICE "/dev/video0"          // Use the appropriate device
#define WIDTH  640
#define HEIGHT 514                  // Height for Boson640 (includes telemetry rows in raw mode)
 
// The node supports two modes:
//   Raw mode: Requests V4L2_PIX_FMT_Y16 and publishes a 16-bit ("mono16") image.
//   AGC mode: Requests V4L2_PIX_FMT_YVU420 (internal AGC output), converts YUV to BGR, and publishes a "bgr8" image.
class BosonCameraNode : public rclcpp::Node {
public:
  // use_agc: if true, use internal AGC (8-bit) mode; otherwise, use raw 16-bit mode.
  BosonCameraNode(bool use_agc) : Node("boson_camera_node"), use_agc_(use_agc) {
    if (use_agc_) {
      pub_ = this->create_publisher<sensor_msgs::msg::Image>("thermal_image_viewer", 10);
    } else {
      pub_ = this->create_publisher<sensor_msgs::msg::Image>("thermal_image_16", 10);
    }
    open_camera();
  }

  ~BosonCameraNode() {
    close_camera();
  }

  // Capture loop: continuously acquires frames and publishes them.
  void capture_frames() {
    while (rclcpp::ok()) {
      if (ioctl(fd_, VIDIOC_QBUF, &bufferinfo_) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to queue buffer");
        continue;
      }
      if (ioctl(fd_, VIDIOC_DQBUF, &bufferinfo_) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to dequeue buffer");
        continue;
      }
      sensor_msgs::msg::Image msg;
      msg.header.stamp = this->get_clock()->now();
      msg.header.frame_id = "boson640";

      if (!use_agc_) {
        // Raw mode: Create a Mat for a 16-bit image of size HEIGHT x WIDTH.
        Mat thermal_raw(HEIGHT, WIDTH, CV_16U, buffer_start_);
        msg.height = HEIGHT;
        msg.width = WIDTH;
        msg.encoding = "mono16";
        msg.is_bigendian = 0;
        msg.step = WIDTH * 2; // 2 bytes per pixel
        msg.data.assign((uint8_t*)thermal_raw.data,
                        (uint8_t*)thermal_raw.data + (WIDTH * HEIGHT * 2));
      } else {
        // AGC mode: The camera outputs 8-bit YUV 4:2:0.
        // For YUV 4:2:0, the buffer size is width x (height + height/2).
        int yuv_height = HEIGHT + HEIGHT / 2;
        Mat thermal_yuv(yuv_height, WIDTH, CV_8UC1, buffer_start_);
        // Convert YUV to BGR.
        Mat thermal_bgr;
        cvtColor(thermal_yuv, thermal_bgr, COLOR_YUV2BGR_I420);
        msg.height = thermal_bgr.rows;
        msg.width = thermal_bgr.cols;
        msg.encoding = "bgr8";
        msg.is_bigendian = 0;
        msg.step = thermal_bgr.cols * 3;
        msg.data.assign((uint8_t*)thermal_bgr.data,
                        (uint8_t*)thermal_bgr.data + (thermal_bgr.rows * thermal_bgr.cols * 3));
      }
      pub_->publish(msg);
    }
  }

  // Close the camera: Stop streaming, unmap the buffer, and close the file descriptor.
  void close_camera() {
    if (fd_ >= 0) {
      if (ioctl(fd_, VIDIOC_STREAMOFF, &type_) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to stop streaming");
      }
      munmap(buffer_start_, bufferinfo_.length);
      close(fd_);
      fd_ = -1;
    }
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  int fd_ = -1;
  int type_ = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  void* buffer_start_;
  struct v4l2_buffer bufferinfo_;
  bool use_agc_;

  // Open the camera, set the desired format, request buffers, and start streaming.
  void open_camera() {
    fd_ = open(DEVICE, O_RDWR);
    if (fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open device %s", DEVICE);
      rclcpp::shutdown();
      return;
    }
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = type_;
    fmt.fmt.pix.width = WIDTH;
    fmt.fmt.pix.height = HEIGHT;
    if (!use_agc_) {
      // Raw 16-bit mode.
      fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_Y16;
    } else {
      // Internal AGC (8-bit) mode.
      fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YVU420;
    }
    if (ioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set video format");
      close(fd_);
      rclcpp::shutdown();
      return;
    }
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = 1;
    req.type = type_;
    req.memory = V4L2_MEMORY_MMAP;
    if (ioctl(fd_, VIDIOC_REQBUFS, &req) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to request buffers");
      close(fd_);
      rclcpp::shutdown();
      return;
    }
    memset(&bufferinfo_, 0, sizeof(bufferinfo_));
    bufferinfo_.type = type_;
    bufferinfo_.memory = V4L2_MEMORY_MMAP;
    bufferinfo_.index = 0;
    if (ioctl(fd_, VIDIOC_QUERYBUF, &bufferinfo_) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to query buffer");
      close(fd_);
      rclcpp::shutdown();
      return;
    }
    buffer_start_ = mmap(NULL, bufferinfo_.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, bufferinfo_.m.offset);
    if (buffer_start_ == MAP_FAILED) {
      RCLCPP_ERROR(this->get_logger(), "Failed to mmap");
      close(fd_);
      rclcpp::shutdown();
      return;
    }
    if (ioctl(fd_, VIDIOC_STREAMON, &type_) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to start streaming");
      close(fd_);
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(this->get_logger(), "FLIR Boson640 camera streaming started.");
  }
};

// Global pointer for signal handling.
std::shared_ptr<BosonCameraNode> node_ptr;

// Signal handler for graceful shutdown.
void signal_handler(int /*signum*/) {
  RCLCPP_INFO(rclcpp::get_logger("boson_camera"), "Received SIGINT, shutting down...");
  if (node_ptr) {
    node_ptr->close_camera();
    node_ptr.reset();
  }
  rclcpp::shutdown();
  exit(0);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  bool use_agc = false;
  // If a command-line parameter is provided and its first character is 'y' or 'Y',
  // then select internal AGC (8-bit) mode.
  if (argc > 1) {
    char mode = argv[1][0];
    if (mode == 'y' || mode == 'Y') {
      use_agc = true;
    }
  }
  node_ptr = std::make_shared<BosonCameraNode>(use_agc);
  signal(SIGINT, signal_handler);
  node_ptr->capture_frames();  // Runs the capture loop indefinitely.
  return 0;
}

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <csignal>  // For handling SIGINT (Ctrl+C)

using namespace cv;

#define DEVICE "/dev/video0"  // Change if needed
#define WIDTH  640
#define HEIGHT 514
#define PIXEL_FORMAT V4L2_PIX_FMT_Y16  // RAW16

class BosonCameraNode : public rclcpp::Node {
public:
    BosonCameraNode() : Node("boson_camera") {
        pub_ = this->create_publisher<sensor_msgs::msg::Image>("thermal_image_16", 10);
        open_camera();
    }

    ~BosonCameraNode() {
        close_camera();
    }

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

            Mat thermal_image(HEIGHT, WIDTH, CV_16U, buffer_start_);

            auto msg = std::make_shared<sensor_msgs::msg::Image>();
            msg->header.stamp = this->get_clock()->now();
            msg->header.frame_id = "thermal_camera";
            msg->height = HEIGHT;
            msg->width = WIDTH;
            msg->encoding = "mono16";  // 16-bit grayscale
            msg->is_bigendian = 0;
            msg->step = WIDTH * 2;  // 2 bytes per pixel
            msg->data.assign((uint8_t *)thermal_image.data, (uint8_t *)thermal_image.data + (WIDTH * HEIGHT * 2));

            pub_->publish(*msg);
        }
    }

    void close_camera() {
        if( ioctl(fd_, VIDIOC_STREAMOFF, &type_) < 0 ){
            RCLCPP_INFO(this->get_logger(), "sum ting wong");
            exit(1);
        }
        close(fd_);

    }

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    int fd_ = -1;
    int type_ = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    void *buffer_start_;
    struct v4l2_buffer bufferinfo_;

    void open_camera() {
        fd_ = open(DEVICE, O_RDWR);
        if (fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open %s", DEVICE);
            rclcpp::shutdown();
            return;
        }

        struct v4l2_format format;
        format.type = type_;
        format.fmt.pix.width = WIDTH;
        format.fmt.pix.height = HEIGHT;
        format.fmt.pix.pixelformat = PIXEL_FORMAT;

        if (ioctl(fd_, VIDIOC_S_FMT, &format) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set format");
            close(fd_);
            rclcpp::shutdown();
            return;
        }

        struct v4l2_requestbuffers bufrequest;
        bufrequest.type = type_;
        bufrequest.memory = V4L2_MEMORY_MMAP;
        bufrequest.count = 1;
        if (ioctl(fd_, VIDIOC_REQBUFS, &bufrequest) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to request buffer");
            close(fd_);
            rclcpp::shutdown();
            return;
        }

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
            RCLCPP_ERROR(this->get_logger(), "Memory mapping failed");
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

        RCLCPP_INFO(this->get_logger(), "FLIR Boson camera streaming started.");
    }
};

// Global pointer for handling shutdown
std::shared_ptr<BosonCameraNode> node_ptr;

void signal_handler(int signum) {
    RCLCPP_INFO(rclcpp::get_logger("boson_camera"), "Received SIGINT (Ctrl+C), shutting down...");
    if (node_ptr) {
        node_ptr->close_camera();
    }
    rclcpp::shutdown();
    exit(0);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    node_ptr = std::make_shared<BosonCameraNode>();

    // Register signal handler for Ctrl+C (SIGINT)
    signal(SIGINT, signal_handler);

    node_ptr->capture_frames();  // Infinite loop, no frame rate control

    return 0;
}

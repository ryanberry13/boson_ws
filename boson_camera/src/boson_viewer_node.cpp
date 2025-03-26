#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;

class BosonViewerNode : public rclcpp::Node {
public:
    BosonViewerNode() : Node("boson_viewer_node") {
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "ir_throttle", 10,
            std::bind(&BosonViewerNode::process_image, this, std::placeholders::_1));

        pub_ = this->create_publisher<sensor_msgs::msg::Image>("thermal_image_viewer", 10);

        RCLCPP_INFO(this->get_logger(), "Boson Viewer Node Started.");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;

    void process_image(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (msg->encoding != "mono16") {
            RCLCPP_ERROR(this->get_logger(), "Received image is not mono16! Skipping...");
            return;
        }

        int width = msg->width;
        int height = msg->height;

        if (height < 514) {
            RCLCPP_ERROR(this->get_logger(), "Image height too small! Expected at least 514 rows, got %d", height);
            return;
        }

        // Convert ROS2 image message to OpenCV Mat (16-bit)
        Mat thermal16(height, width, CV_16U, const_cast<uint8_t*>(msg->data.data()));

        // Remove first 2 rows (telemetry)
        Mat thermal16_no_telemetry = thermal16(Range(2, height), Range::all()).clone();

        // Perform AGC (Auto-Gain Control)
        Mat thermal8 = Mat::zeros(thermal16_no_telemetry.size(), CV_8U);
        AGC_Basic_Linear(thermal16_no_telemetry, thermal8, thermal16_no_telemetry.rows, width);

        // Publish the processed 8-bit image
        auto msg_out = std::make_shared<sensor_msgs::msg::Image>();
        msg_out->header = msg->header;
        msg_out->height = thermal8.rows;
        msg_out->width = thermal8.cols;
        msg_out->encoding = "mono8";  // 8-bit grayscale
        msg_out->is_bigendian = 0;
        msg_out->step = width;
        msg_out->data.assign(thermal8.data, thermal8.data + (thermal8.rows * thermal8.cols));

        pub_->publish(*msg_out);
    }

    // AGC FUNCTION: Performs Linear Auto-Gain Control (Basic)
    void AGC_Basic_Linear(Mat &input_16, Mat &output_8, int height, int width) {
        unsigned int max1 = 0;
        unsigned int min1 = 0xFFFF;
        unsigned int value1, value2, value3, value4;

        // Find min and max pixel values
        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                value1 = input_16.at<uchar>(i, j * 2 + 1) & 0XFF;  // High Byte
                value2 = input_16.at<uchar>(i, j * 2) & 0xFF;      // Low Byte
                value3 = (value1 << 8) + value2;

                if (value3 <= min1) min1 = value3;
                if (value3 >= max1) max1 = value3;
            }
        }

        // Normalize to 8-bit range
        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                value1 = input_16.at<uchar>(i, j * 2 + 1) & 0XFF;
                value2 = input_16.at<uchar>(i, j * 2) & 0xFF;
                value3 = (value1 << 8) + value2;
                value4 = ((255 * (value3 - min1)) / (max1 - min1));
                output_8.at<uchar>(i, j) = static_cast<uchar>(value4 & 0xFF);
            }
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BosonViewerNode>());
    rclcpp::shutdown();
    return 0;
}

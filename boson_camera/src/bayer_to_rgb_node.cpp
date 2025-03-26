#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

class BayerToRGBNode : public rclcpp::Node {
public:
    BayerToRGBNode() : Node("bayer_to_rgb_node") {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "rgb_throttle", 10, std::bind(&BayerToRGBNode::imageCallback, this, std::placeholders::_1));
        
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("rgb_image", 10);
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // Convert the ROS image message to an OpenCV image
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bayer_rggb8");
            
            cv::Mat rgb_image;
            // Convert Bayer RG8 to RGB
            cv::cvtColor(cv_ptr->image, rgb_image, cv::COLOR_BayerBG2RGB);
            
            // Convert back to ROS message
            auto rgb_msg = cv_bridge::CvImage(msg->header, "rgb8", rgb_image).toImageMsg();
            
            // Publish the RGB image
            image_pub_->publish(*rgb_msg);
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BayerToRGBNode>());
    rclcpp::shutdown();
    return 0;
}

#include<rclcpp/rclcpp.hpp>
#include<opencv2/opencv.hpp>
#include<sensor_msgs/msg/image.hpp>
#include<cv_bridge/cv_bridge.h>

class LaneDetector : public rclcpp::Node {
    public:
        LaneDetector() : Node("lane_detector") {
            image_sub = this->create_subscription<sensor_msgs::msg::Image>(
                "/camera/image_raw", 10, 
                std::bind(&LaneDetector::image_callback, this,  std::placeholders::_1)
            );

            debug_img_pub = this->create_publisher<sensor_msgs::msg::Image>("/lane_mask", 10);
        }
    
    private:
        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
            cv::Mat bgr_image = cv_bridge::toCvShare(msg, "bgr8")->image;

            cv::GaussianBlur(
                bgr_image,
                bgr_image,
                cv::Size(5,5),
                0
            );

            cv::Mat hsv;
            cv::cvtColor(bgr_image, hsv, cv::COLOR_BGR2HSV);

            cv::Mat white_mask, yellow_mask;
            cv::inRange(hsv, cv::Scalar(0,0,140), cv::Scalar(180,80,255), white_mask);
            cv::inRange(hsv, cv::Scalar(15,50,50), cv::Scalar(35,255,255), yellow_mask);

            cv::Mat lane_mask;
            cv::bitwise_or(white_mask, yellow_mask, lane_mask);

            // Morphological closing
            cv::Mat kernel =
                cv::getStructuringElement(
                    cv::MORPH_RECT,
                    cv::Size(5,5)
            );

            cv::morphologyEx(
                lane_mask,
                lane_mask,
                cv::MORPH_CLOSE,
                kernel
            );

            auto debug_img_msg = cv_bridge::CvImage(msg->header, "mono8", lane_mask).toImageMsg();
            debug_img_pub->publish(*debug_img_msg);

            RCLCPP_INFO(this->get_logger(), "Lane mask ");

        }
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_img_pub;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaneDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
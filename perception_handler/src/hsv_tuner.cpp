#include <rclcpp/rclcpp.hpp>

#include <opencv2/opencv.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.h>

class HSVTuner : public rclcpp::Node
{
public:
    HSVTuner() : Node("hsv_tuner")
    {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw",
            10,
            std::bind(&HSVTuner::imageCallback, this, std::placeholders::_1));

        // ================= WINDOWS =================
        cv::namedWindow("Original", cv::WINDOW_NORMAL);
        cv::namedWindow("HSV", cv::WINDOW_NORMAL);
        cv::namedWindow("White Mask", cv::WINDOW_NORMAL);
        cv::namedWindow("Yellow Mask", cv::WINDOW_NORMAL);
        cv::namedWindow("Combined Mask", cv::WINDOW_NORMAL);

        // ================= WHITE TRACKBARS =================
        cv::createTrackbar("W_H_MIN", "White Mask", &w_h_min_, 180);
        cv::createTrackbar("W_S_MIN", "White Mask", &w_s_min_, 255);
        cv::createTrackbar("W_V_MIN", "White Mask", &w_v_min_, 255);

        cv::createTrackbar("W_H_MAX", "White Mask", &w_h_max_, 180);
        cv::createTrackbar("W_S_MAX", "White Mask", &w_s_max_, 255);
        cv::createTrackbar("W_V_MAX", "White Mask", &w_v_max_, 255);

        // ================= YELLOW TRACKBARS =================
        cv::createTrackbar("Y_H_MIN", "Yellow Mask", &y_h_min_, 180);
        cv::createTrackbar("Y_S_MIN", "Yellow Mask", &y_s_min_, 255);
        cv::createTrackbar("Y_V_MIN", "Yellow Mask", &y_v_min_, 255);

        cv::createTrackbar("Y_H_MAX", "Yellow Mask", &y_h_max_, 180);
        cv::createTrackbar("Y_S_MAX", "Yellow Mask", &y_s_max_, 255);
        cv::createTrackbar("Y_V_MAX", "Yellow Mask", &y_v_max_, 255);

        RCLCPP_INFO(this->get_logger(), "HSV Tuner Started");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat bgr =
            cv_bridge::toCvShare(msg, "bgr8")->image;

        if (bgr.empty())
            return;

        // ================= PREPROCESS =================
        cv::Mat blurred;
        cv::GaussianBlur(bgr, blurred, cv::Size(5,5), 0);

        cv::Mat hsv;
        cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);

        // ================= MASKS =================
        cv::Mat white_mask;
        cv::inRange(
            hsv,
            cv::Scalar(w_h_min_, w_s_min_, w_v_min_),
            cv::Scalar(w_h_max_, w_s_max_, w_v_max_),
            white_mask);

        cv::Mat yellow_mask;
        cv::inRange(
            hsv,
            cv::Scalar(y_h_min_, y_s_min_, y_v_min_),
            cv::Scalar(y_h_max_, y_s_max_, y_v_max_),
            yellow_mask);

        // ================= COMBINE =================
        cv::Mat combined_mask;
        cv::bitwise_or(white_mask, yellow_mask, combined_mask);

        // ================= MORPH CLOSE =================
        cv::Mat kernel =
            cv::getStructuringElement(
                cv::MORPH_RECT,
                cv::Size(5,5));

        cv::morphologyEx(
            combined_mask,
            combined_mask,
            cv::MORPH_CLOSE,
            kernel);

        // ================= DISPLAY =================
        cv::imshow("Original", bgr);

        cv::imshow("HSV", hsv);

        cv::imshow("White Mask", white_mask);

        cv::imshow("Yellow Mask", yellow_mask);

        cv::imshow("Combined Mask", combined_mask);

        cv::waitKey(1);

        // ================= DEBUG PRINT =================
        static int counter = 0;

        if(counter++ % 50 == 0)
        {
            RCLCPP_INFO(this->get_logger(),
                "\nWHITE:\n"
                "H:[%d,%d] S:[%d,%d] V:[%d,%d]\n"
                "YELLOW:\n"
                "H:[%d,%d] S:[%d,%d] V:[%d,%d]",

                w_h_min_, w_h_max_,
                w_s_min_, w_s_max_,
                w_v_min_, w_v_max_,

                y_h_min_, y_h_max_,
                y_s_min_, y_s_max_,
                y_v_min_, y_v_max_);
        }
    }

    // ================= ROS =================
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    // ================= WHITE HSV =================
    int w_h_min_ = 0;
    int w_s_min_ = 0;
    int w_v_min_ = 150;

    int w_h_max_ = 180;
    int w_s_max_ = 70;
    int w_v_max_ = 255;

    // ================= YELLOW HSV =================
    int y_h_min_ = 15;
    int y_s_min_ = 50;
    int y_v_min_ = 50;

    int y_h_max_ = 40;
    int y_s_max_ = 255;
    int y_v_max_ = 255;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<HSVTuner>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
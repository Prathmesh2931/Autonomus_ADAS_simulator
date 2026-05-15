#include<rclcpp/rclcpp.hpp>
#include<opencv2/opencv.hpp>
#include<sensor_msgs/msg/image.hpp>
#include<cv_bridge/cv_bridge.h>
#include <deque>
#include <numeric>


class LaneDetector : public rclcpp::Node {
    public:
        LaneDetector() : Node("lane_detector") {
            image_sub = this->create_subscription<sensor_msgs::msg::Image>(
                "/camera/image_raw", 10, 
                std::bind(&LaneDetector::image_callback, this, std::placeholders::_1)
            );

            debug_img_pub = this->create_publisher<sensor_msgs::msg::Image>("/lane_mask", 10);
            roi_display_pub = this->create_publisher<sensor_msgs::msg::Image>("/roi_debug", 10);
            hough_debug_pub = this->create_publisher<sensor_msgs::msg::Image>("/hough_debug", 10);
            
            // Optional: Subscribe to get current speed
            // speed_sub = this->create_subscription<geometry_msgs::msg::Twist>(
            //     "/cmd_vel", 10, 
            //     std::bind(&LaneDetector::speed_callback, this, std::placeholders::_1));
        }
    
    private:

        cv::Vec3d fitPolynomial(const std::vector<cv::Point>& points)
        {
            if (points.size() < 6)
                return cv::Vec3d(0, 0, 0);          // not enough data → return zeros
 
            cv::Mat1d X(points.size(), 3);
            cv::Mat1d Y(points.size(), 1);
            for (size_t i = 0; i < points.size(); i++) {
                double y = points[i].y;
                double x = points[i].x;
                X(i, 0) = y * y;
                X(i, 1) = y;
                X(i, 2) = 1;
                Y(i, 0) = x;
            }
            cv::Mat1d coeffs;
            cv::solve(X, Y, coeffs, cv::DECOMP_SVD);
            return cv::Vec3d(coeffs(0,0), coeffs(1,0), coeffs(2,0));
        }

        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
            cv::Mat bgr_image = cv_bridge::toCvShare(msg, "bgr8")->image;
            cv::Mat original_image = bgr_image.clone();

            cv::GaussianBlur(bgr_image, bgr_image, cv::Size(5,5), 0);

            cv::Mat hsv;
            cv::cvtColor(bgr_image, hsv, cv::COLOR_BGR2HSV);

            cv::Mat white_mask, yellow_mask;
            cv::inRange(hsv, cv::Scalar(0,0,140), cv::Scalar(180,80,255), white_mask);
            cv::inRange(hsv, cv::Scalar(15,50,50), cv::Scalar(35,255,255), yellow_mask);

            cv::Mat lane_mask;
            cv::bitwise_or(white_mask, yellow_mask, lane_mask);

            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5));
            cv::morphologyEx(lane_mask, lane_mask, cv::MORPH_CLOSE, kernel);

            // ========== IMPROVED ROI ==========
            cv::Mat roi_mask = cv::Mat::zeros(lane_mask.size(), lane_mask.type());
            
            int height = lane_mask.rows;
            int width = lane_mask.cols;
            
            // Dynamic look-ahead based on speed (if you have speed data)
            double look_ahead = 0.2;  // Default: look 60% down the image
            // if (current_speed_ > 0.5) look_ahead = 0.5;  // Look further at higher speeds
            
            // Wider trapezoid for better curve handling
            std::vector<cv::Point> roi_points;
            roi_points.push_back(cv::Point(width* 0.2, height));                       // bottom-left
            roi_points.push_back(cv::Point(width * 0.1, height * look_ahead)); // top-left (narrower)
            roi_points.push_back(cv::Point(width * 0.9, height * look_ahead)); // top-right (narrower)
            roi_points.push_back(cv::Point(width* 0.8, height));                   // bottom-right
            
            cv::fillPoly(roi_mask, roi_points, cv::Scalar(255));
            cv::bitwise_and(lane_mask, roi_mask, lane_mask);
            
            // Debug visualization
            cv::Mat debug_roi = original_image.clone();
            cv::polylines(debug_roi, roi_points, true, cv::Scalar(0, 255, 0), 2);
            
            // Optional: Draw robot exclusion zone (bottom 10%)
            cv::rectangle(debug_roi, 
                         cv::Point(0, height * 0.92), 
                         cv::Point(width, height), 
                         cv::Scalar(0, 0, 255), -1);  // Red zone - ignore
            
            
            // Canny edge detection for better lane line visualization 
            cv::Mat edges;
            cv::Canny(lane_mask, edges, 50, 150);

            // Hough Transform for line detection 
            std::vector<cv::Vec4i> lines;
            cv::HoughLinesP(edges, lines, 1, CV_PI/180, 80, 50, 20);

            // Draw detected lines on debug image
            cv::Mat hough_debug = original_image.clone();
            int left_count = 0, right_count = 0;

            std::vector<cv::Point> left_points;
            std::vector<cv::Point> right_points;


            for (const auto& line : lines) {
                int x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];

                if (x2 - x1 == 0) continue;
                float slope = (float)(y2 -y1) / (x2 - x1);

                // filter out near horizontal lines 
                if(std::abs(slope) < 0.3) continue;

                cv::Scalar line_color;
                if (slope < 0 ) {
                    line_color = cv::Scalar(0, 0, 255); // Left lane - blue
                    left_count++;
                    left_points.push_back(cv::Point(x1,y1));
                    left_points.push_back(cv::Point(x2,y2));

                } else {
                    line_color = cv::Scalar(255, 0, 0); // Right lane - green
                    right_count++;
                    right_points.push_back(cv::Point(x1,y1));
                    right_points.push_back(cv::Point(x2,y2));
                }

                cv::line(hough_debug, cv::Point(x1, y1), cv::Point(x2, y2), line_color, 2);
            }

            //  NEW: fit one polynomial per side 
            cv::Vec3d left_curve  = fitPolynomial(left_points);
            cv::Vec3d right_curve = fitPolynomial(right_points);
 
            //  NEW: draw smooth curves on TOP of the existing hough_debug ─
            //         cyan  = left   |   green = right
            for (int y = (int)(height * 0.2); y < height; y += 5)
            {
                double lx = left_curve[0]*y*y  + left_curve[1]*y  + left_curve[2];
                double rx = right_curve[0]*y*y + right_curve[1]*y + right_curve[2];
 
                if (lx > 0 && lx < width)
                    cv::circle(hough_debug, cv::Point((int)lx, y), 3,
                               cv::Scalar(0, 255, 255), -1);   // cyan
 
                if (rx > 0 && rx < width)
                    cv::circle(hough_debug, cv::Point((int)rx, y), 3,
                               cv::Scalar(0, 255, 0),   -1);   // green
            }


            auto debug_img_msg = cv_bridge::CvImage(msg->header, "mono8", lane_mask).toImageMsg();
            debug_img_pub->publish(*debug_img_msg);
            
            auto roi_debug_msg = cv_bridge::CvImage(msg->header, "bgr8", debug_roi).toImageMsg();
            roi_display_pub->publish(*roi_debug_msg);

            auto hough_debug_msg = cv_bridge::CvImage(msg->header, "bgr8", hough_debug).toImageMsg();
            hough_debug_pub->publish(*hough_debug_msg);

            RCLCPP_INFO(this->get_logger(),
            "Left pts: %ld  Right pts: %ld  |  Lane pixels: %.2f%%",
            left_points.size(), right_points.size(),
            cv::countNonZero(lane_mask) / (double)lane_mask.total() * 100);

        }
        
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_img_pub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr roi_display_pub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr hough_debug_pub;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaneDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
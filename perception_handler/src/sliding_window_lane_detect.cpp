#include<rclcpp/rclcpp.hpp>
#include<opencv2/opencv.hpp>
#include<sensor_msgs/msg/image.hpp>
#include<cv_bridge/cv_bridge.h>
#include <deque>
#include <numeric>
#include <vector>
#include <algorithm>

class LaneDetector : public rclcpp::Node {
    public:
        LaneDetector() : Node("lane_detector") {
            image_sub = this->create_subscription<sensor_msgs::msg::Image>(
                "/camera/image_raw", 10, 
                std::bind(&LaneDetector::image_callback, this, std::placeholders::_1)
            );

            debug_img_pub = this->create_publisher<sensor_msgs::msg::Image>("/lane_mask", 10);
            bev_pub = this->create_publisher<sensor_msgs::msg::Image>("/birds_eye", 10);
            curve_pub = this->create_publisher<sensor_msgs::msg::Image>("/curve_debug", 10);
            roi_display_pub = this->create_publisher<sensor_msgs::msg::Image>("/roi_debug", 10);
        }
    
    private:
        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
            //  STEP 0: CONVERT AND PREPROCESS 
            cv::Mat bgr_image = cv_bridge::toCvShare(msg, "bgr8")->image;
            cv::Mat original_image = bgr_image.clone();
            
            int height = bgr_image.rows;
            int width = bgr_image.cols;

            // Blur
            cv::GaussianBlur(bgr_image, bgr_image, cv::Size(5,5), 0);

            // Color filtering
            cv::Mat hsv;
            cv::cvtColor(bgr_image, hsv, cv::COLOR_BGR2HSV);

            cv::Mat white_mask, yellow_mask;
            cv::inRange(hsv, cv::Scalar(0, 0, 160), cv::Scalar(180, 80, 255), white_mask);
            cv::inRange(hsv, cv::Scalar(15, 80, 80), cv::Scalar(40, 255, 255), yellow_mask);

            cv::Mat lane_mask;
            cv::bitwise_or(white_mask, yellow_mask, lane_mask);

            // Morphological closing
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5));
            cv::morphologyEx(lane_mask, lane_mask, cv::MORPH_CLOSE, kernel);

            //  STEP 1: REGION OF INTEREST 
            cv::Mat roi_mask = cv::Mat::zeros(lane_mask.size(), lane_mask.type());
            
            std::vector<cv::Point> roi_points;
            roi_points.push_back(cv::Point(width * 0.01, height));
            roi_points.push_back(cv::Point(width * 0.10, height * 0.35));
            roi_points.push_back(cv::Point(width * 0.90, height * 0.35));
            roi_points.push_back(cv::Point(width * 0.99, height));
            
            cv::fillPoly(roi_mask, roi_points, cv::Scalar(255));
            cv::bitwise_and(lane_mask, roi_mask, lane_mask);
            
            // Debug ROI visualization
            cv::Mat debug_roi = original_image.clone();
            cv::polylines(debug_roi, roi_points, true, cv::Scalar(0, 255, 0), 2);
            auto roi_debug_msg = cv_bridge::CvImage(msg->header, "bgr8", debug_roi).toImageMsg();
            roi_display_pub->publish(*roi_debug_msg);

            //  STEP 2: BIRD'S EYE TRANSFORM 
            cv::Mat birds_eye, perspective_transform, inverse_transform;

            // Source points (trapezoid ROI)
            std::vector<cv::Point2f> src_pts;
            src_pts.push_back(cv::Point2f(width * 0.15, height));
            src_pts.push_back(cv::Point2f(width * 0.45, height * 0.35));
            src_pts.push_back(cv::Point2f(width * 0.55, height * 0.35));
            src_pts.push_back(cv::Point2f(width * 0.85, height));

            // Destination points (rectangle in BEV)
            std::vector<cv::Point2f> dst_pts;
            dst_pts.push_back(cv::Point2f(width * 0.25, height));
            dst_pts.push_back(cv::Point2f(width * 0.25, 0));
            dst_pts.push_back(cv::Point2f(width * 0.75, 0));
            dst_pts.push_back(cv::Point2f(width * 0.75, height));

            // Calculate and apply transform
            perspective_transform = cv::getPerspectiveTransform(src_pts, dst_pts);
            inverse_transform = cv::getPerspectiveTransform(dst_pts, src_pts);
            cv::warpPerspective(lane_mask, birds_eye, perspective_transform, 
                                cv::Size(width, height));

            //  STEP 3: HISTOGRAM 
            std::vector<int> histogram(width, 0);
            int bottom_start = height * 0.7;

            for (int y = bottom_start; y < height; y++) {
                const uchar* row = birds_eye.ptr<uchar>(y);
                for (int x = 0; x < width; x++) {
                    if (row[x] > 0) {
                        histogram[x]++;
                    }
                }
            }

            // Find peaks
            int midpoint = width / 2;
            int left_base = std::max_element(histogram.begin(), histogram.begin() + midpoint) - histogram.begin();
            int right_base = std::max_element(histogram.begin() + midpoint, histogram.end()) - histogram.begin();

            cv::Mat hist_img(height, width, CV_8UC3, cv::Scalar(0,0,0));

            int max_value = *std::max_element(histogram.begin(), histogram.end());

            for (int x = 0; x < width; x++)
            {
                int h = ((double)histogram[x] / max_value) * height;

                cv::line(hist_img,
                        cv::Point(x, height),
                        cv::Point(x, height - h),
                        cv::Scalar(255,255,255),
                        2);
            }

            cv::circle(hist_img,
                    cv::Point(left_base, height-20),
                    10,
                    cv::Scalar(0,0,255),
                    -1);

            cv::circle(hist_img,
                    cv::Point(right_base, height-20),
                    10,
                    cv::Scalar(255,0,0),
                    -1);

            cv::imshow("Histogram", hist_img);
            cv::namedWindow("Histogram", cv::WINDOW_NORMAL);
            cv::resizeWindow("Histogram", 800, 600);

            cv::imshow("Histogram", hist_img);
            cv::waitKey(1);

            //  STEP 4: SLIDING WINDOWS 
            std::vector<cv::Point> left_pixels, right_pixels;
            int n_windows = 9;
            int window_height = height / n_windows;
            int margin = 80;
            int min_pixels = 25;

            int left_x = left_base;
            int right_x = right_base;

            cv::Mat sliding_debug;
            cv::cvtColor(birds_eye, sliding_debug, cv::COLOR_GRAY2BGR); 

            for (int w = 0; w < n_windows; w++) {
                int y_low = height - (w + 1) * window_height;
                
                // LEFT WINDOW
                int left_win_x = std::max(0, left_x - margin);
                int left_win_width = std::min(margin * 2, width - left_win_x);
                cv::Rect left_win(left_win_x, y_low, left_win_width, window_height);

                cv::rectangle(sliding_debug, left_win, cv::Scalar(0, 255, 0), 2);
                
                cv::Mat left_roi = birds_eye(left_win);
                std::vector<cv::Point> left_indices;
                cv::findNonZero(left_roi, left_indices);
                
                for (auto& idx : left_indices) {
                    cv::Point left_pt(idx.x + left_win_x, idx.y + y_low);  
                    left_pixels.push_back(left_pt);
                    cv::circle(sliding_debug, left_pt, 2, cv::Scalar(0, 0, 255), -1);
                }
                
                if (left_indices.size() > (size_t)min_pixels) {
                    int sum_x = 0;
                    for (auto& idx : left_indices) sum_x += idx.x;
                    left_x = left_win_x + (sum_x / (int)left_indices.size());
                }
                
                // RIGHT WINDOW
                int right_win_x = std::max(0, right_x - margin);
                int right_win_width = std::min(margin * 2, width - right_win_x);
                cv::Rect right_win(right_win_x, y_low, right_win_width, window_height);
                cv::rectangle(sliding_debug, right_win, cv::Scalar(255, 0, 0), 2);

                cv::Mat right_roi = birds_eye(right_win);
                std::vector<cv::Point> right_indices;
                cv::findNonZero(right_roi, right_indices);
                
                for (auto& idx : right_indices) {
                    cv::Point right_pt(idx.x + right_win_x, idx.y + y_low);  
                    right_pixels.push_back(right_pt);
                    cv::circle(sliding_debug, right_pt, 2, cv::Scalar(0, 255, 255), -1);
                }
                
                if (right_indices.size() > (size_t)min_pixels) {
                    int sum_x = 0;
                    for (auto& idx : right_indices) sum_x += idx.x;
                    right_x = right_win_x + (sum_x / (int)right_indices.size());
                }
            }

            cv::namedWindow("Sliding Window Debug", cv::WINDOW_NORMAL);
            cv::resizeWindow("Sliding Window Debug", 800, 600);
            cv::imshow("Sliding Window Debug", sliding_debug);
            cv::waitKey(1);

            //  STEP 5: POLYNOMIAL FITTING 
            cv::Mat curve_display = original_image.clone();
            
            // Left lane polynomial
            if (left_pixels.size() > 20) {
                std::vector<cv::Point2f> left_points;
                for (auto& p : left_pixels) {
                    left_points.push_back(cv::Point2f(p.x, p.y));
                }
                
                // Sort by y
                std::sort(left_points.begin(), left_points.end(), 
                         [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });
                
                cv::Mat left_coeff = fitPoly(left_points, 2);
                
                if (!left_coeff.empty()) {
                    std::vector<cv::Point2f> bev_curve;
                    for (int y = 0; y < height; y += 5) {
                        float x = left_coeff.at<float>(2) * y * y + 
                                 left_coeff.at<float>(1) * y + 
                                 left_coeff.at<float>(0);
                        if (x >= 0 && x < width) {
                            bev_curve.push_back(cv::Point2f(x, y));
                        }
                    }
                    
                    if (!bev_curve.empty()) {
                        std::vector<cv::Point2f> orig_curve;
                        cv::perspectiveTransform(bev_curve, orig_curve, inverse_transform);
                        
                        for (size_t i = 1; i < orig_curve.size(); i++) {
                            cv::line(curve_display, orig_curve[i-1], orig_curve[i], 
                                    cv::Scalar(0, 0, 255), 5);
                        }
                    }
                }
            }
            
            // Right lane polynomial
            if (right_pixels.size() > 20) {
                std::vector<cv::Point2f> right_points;
                for (auto& p : right_pixels) {
                    right_points.push_back(cv::Point2f(p.x, p.y));
                }
                
                std::sort(right_points.begin(), right_points.end(),
                         [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });
                
                cv::Mat right_coeff = fitPoly(right_points, 2);
                
                if (!right_coeff.empty()) {
                    std::vector<cv::Point2f> bev_curve;
                    for (int y = 0; y < height; y += 5) {
                        float x = right_coeff.at<float>(2) * y * y + 
                                 right_coeff.at<float>(1) * y + 
                                 right_coeff.at<float>(0);
                        if (x >= 0 && x < width) {
                            bev_curve.push_back(cv::Point2f(x, y));
                        }
                    }
                    
                    if (!bev_curve.empty()) {
                        std::vector<cv::Point2f> orig_curve;
                        cv::perspectiveTransform(bev_curve, orig_curve, inverse_transform);
                        
                        for (size_t i = 1; i < orig_curve.size(); i++) {
                            cv::line(curve_display, orig_curve[i-1], orig_curve[i], 
                                    cv::Scalar(255, 0, 0), 5);
                        }
                    }
                }
            }

            //  STEP 6: PUBLISH DEBUG TOPICS 
            auto mask_msg = cv_bridge::CvImage(msg->header, "mono8", lane_mask).toImageMsg();
            debug_img_pub->publish(*mask_msg);
            
            auto bev_msg = cv_bridge::CvImage(msg->header, "mono8", birds_eye).toImageMsg();
            bev_pub->publish(*bev_msg);
            
            auto curve_msg = cv_bridge::CvImage(msg->header, "bgr8", curve_display).toImageMsg();
            curve_pub->publish(*curve_msg);
            
            static int frame_count = 0;
            if (frame_count++ % 30 == 0) {
                RCLCPP_INFO(this->get_logger(), 
                           "Left pixels: %zu, Right pixels: %zu", 
                           left_pixels.size(), right_pixels.size());
            }
        }
        
        // Helper function for polynomial fitting
        cv::Mat fitPoly(const std::vector<cv::Point2f>& points, int degree) {
            int n = points.size();
            if (n < degree + 1) return cv::Mat();
            
            cv::Mat X(n, degree + 1, CV_32F);
            cv::Mat Y(n, 1, CV_32F);
            
            for (int i = 0; i < n; i++) {
                float y = points[i].y;
                float x = points[i].x;
                
                for (int j = 0; j <= degree; j++) {
                    X.at<float>(i, j) = pow(y, j);
                }
                Y.at<float>(i, 0) = x;
            }
            
            cv::Mat coeff;
            cv::solve(X, Y, coeff, cv::DECOMP_QR);
            return coeff;
        }
        
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_img_pub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr roi_display_pub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr bev_pub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr curve_pub;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaneDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
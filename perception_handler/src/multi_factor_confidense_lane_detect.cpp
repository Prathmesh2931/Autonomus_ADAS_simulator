#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cv_bridge/cv_bridge.h>
#include <deque>
#include <numeric>
#include <vector>
#include <algorithm>

class LaneDetector : public rclcpp::Node {
public:
    LaneDetector() : Node("lane_detector") {
        image_sub = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&LaneDetector::image_callback, this, std::placeholders::_1));

        debug_img_pub = this->create_publisher<sensor_msgs::msg::Image>("/lane_mask", 10);
        bev_pub = this->create_publisher<sensor_msgs::msg::Image>("/birds_eye", 10);
        curve_pub = this->create_publisher<sensor_msgs::msg::Image>("/curve_debug", 10);
        roi_display_pub = this->create_publisher<sensor_msgs::msg::Image>("/roi_debug", 10);
        cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Temporal smoothing storage
        prev_left_coeff_ = cv::Mat::zeros(3, 1, CV_32F);
        prev_right_coeff_ = cv::Mat::zeros(3, 1, CV_32F);
        prev_lane_center_ = width_ * 0.5;
        lane_width_history_.resize(10, 100.0f);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // 0. PREPROCESS (unchanged)
        cv::Mat bgr_image = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::Mat original_image = bgr_image.clone();
        int height = bgr_image.rows;
        int width = bgr_image.cols;
        width_ = width;

        cv::GaussianBlur(bgr_image, bgr_image, cv::Size(5,5), 0);

        cv::Mat hsv, gray;
        cv::cvtColor(bgr_image, hsv, cv::COLOR_BGR2HSV);
        cv::cvtColor(bgr_image, gray, cv::COLOR_BGR2GRAY);

        // HSV mask
        cv::Mat white_mask, yellow_mask;
        cv::inRange(hsv, cv::Scalar(0, 0, 160), cv::Scalar(180, 80, 255), white_mask);
        cv::inRange(hsv, cv::Scalar(15, 80, 80), cv::Scalar(40, 255, 255), yellow_mask);
        cv::Mat hsv_mask;
        cv::bitwise_or(white_mask, yellow_mask, hsv_mask);

        // Sobel edge mask
        cv::Mat sobel_x, sobel_mask;
        cv::Sobel(gray, sobel_x, CV_64F, 1, 0, 3);
        sobel_x = cv::abs(sobel_x);
        cv::normalize(sobel_x, sobel_x, 0, 255, cv::NORM_MINMAX);
        sobel_x.convertTo(sobel_x, CV_8U);
        cv::threshold(sobel_x, sobel_mask, 50, 255, cv::THRESH_BINARY);

        // Combine masks
        cv::Mat lane_mask;
        cv::bitwise_or(hsv_mask, sobel_mask, lane_mask);
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5));
        cv::morphologyEx(lane_mask, lane_mask, cv::MORPH_CLOSE, kernel);

        // 1. ROI (YOUR ORIGINAL)
        cv::Mat roi_mask = cv::Mat::zeros(lane_mask.size(), lane_mask.type());
        std::vector<cv::Point> roi_points;
        roi_points.push_back(cv::Point(width * 0.01, height));
        roi_points.push_back(cv::Point(width * 0.10, height * 0.35));
        roi_points.push_back(cv::Point(width * 0.90, height * 0.35));
        roi_points.push_back(cv::Point(width * 0.99, height));
        cv::fillPoly(roi_mask, roi_points, cv::Scalar(255));
        cv::bitwise_and(lane_mask, roi_mask, lane_mask);

        cv::Mat debug_roi = original_image.clone();
        cv::polylines(debug_roi, roi_points, true, cv::Scalar(0, 255, 0), 2);
        auto roi_debug_msg = cv_bridge::CvImage(msg->header, "bgr8", debug_roi).toImageMsg();
        roi_display_pub->publish(*roi_debug_msg);

        // 2. BIRD'S EYE TRANSFORM (YOUR ORIGINAL)
        cv::Mat birds_eye, perspective_transform, inverse_transform;
        std::vector<cv::Point2f> src_pts;
        src_pts.push_back(cv::Point2f(width * 0.15, height));
        src_pts.push_back(cv::Point2f(width * 0.45, height * 0.35));
        src_pts.push_back(cv::Point2f(width * 0.55, height * 0.35));
        src_pts.push_back(cv::Point2f(width * 0.85, height));
        std::vector<cv::Point2f> dst_pts;
        dst_pts.push_back(cv::Point2f(width * 0.25, height));
        dst_pts.push_back(cv::Point2f(width * 0.25, 0));
        dst_pts.push_back(cv::Point2f(width * 0.75, 0));
        dst_pts.push_back(cv::Point2f(width * 0.75, height));
        perspective_transform = cv::getPerspectiveTransform(src_pts, dst_pts);
        inverse_transform = cv::getPerspectiveTransform(dst_pts, src_pts);
        cv::warpPerspective(lane_mask, birds_eye, perspective_transform, cv::Size(width, height));

        // 3. HISTOGRAM & SLIDING WINDOW (YOUR ORIGINAL)
        std::vector<int> histogram(width, 0);
        int bottom_start = height * 0.7;
        for (int y = bottom_start; y < height; ++y) {
            const uchar* row = birds_eye.ptr<uchar>(y);
            for (int x = 0; x < width; ++x)
                if (row[x] > 0) histogram[x]++;
        }
        int midpoint = width / 2;
        int left_base = std::max_element(histogram.begin(), histogram.begin() + midpoint) - histogram.begin();
        int right_base = std::max_element(histogram.begin() + midpoint, histogram.end()) - histogram.begin();

        std::vector<cv::Point> left_pixels, right_pixels;
        int n_windows = 9;
        int window_height = height / n_windows;
        int margin = 80;
        int min_pixels = 25;
        int left_x = left_base, right_x = right_base;

        for (int w = 0; w < n_windows; ++w) {
            int y_low = height - (w + 1) * window_height;
            // Left window
            int left_win_x = std::max(0, left_x - margin);
            int left_win_width = std::min(margin * 2, width - left_win_x);
            cv::Rect left_win(left_win_x, y_low, left_win_width, window_height);
            cv::Mat left_roi = birds_eye(left_win);
            std::vector<cv::Point> left_idx;
            cv::findNonZero(left_roi, left_idx);
            for (auto& idx : left_idx)
                left_pixels.push_back(cv::Point(idx.x + left_win_x, idx.y + y_low));
            if (left_idx.size() > (size_t)min_pixels) {
                int sum = 0;
                for (auto& idx : left_idx) sum += idx.x;
                left_x = left_win_x + sum / (int)left_idx.size();
            }
            // Right window
            int right_win_x = std::max(0, right_x - margin);
            int right_win_width = std::min(margin * 2, width - right_win_x);
            cv::Rect right_win(right_win_x, y_low, right_win_width, window_height);
            cv::Mat right_roi = birds_eye(right_win);
            std::vector<cv::Point> right_idx;
            cv::findNonZero(right_roi, right_idx);
            for (auto& idx : right_idx)
                right_pixels.push_back(cv::Point(idx.x + right_win_x, idx.y + y_low));
            if (right_idx.size() > (size_t)min_pixels) {
                int sum = 0;
                for (auto& idx : right_idx) sum += idx.x;
                right_x = right_win_x + sum / (int)right_idx.size();
            }
        }

        // 4. POLYNOMIAL FITTING
        cv::Mat left_coeff = fitPolyFromPoints(left_pixels, 2);
        cv::Mat right_coeff = fitPolyFromPoints(right_pixels, 2);

        // --- CURVATURE STRENGTH (from second-order coefficient) ---
        float left_curve = 0.0f, right_curve = 0.0f;
        if (!left_coeff.empty()) left_curve = std::abs(left_coeff.at<float>(2));
        if (!right_coeff.empty()) right_curve = std::abs(right_coeff.at<float>(2));
        float curve_strength = std::max(left_curve, right_curve);

        // --- ADAPTIVE SMOOTHING (less smoothing in sharp turns) ---
        float adaptive_alpha = 0.7f;
        if (curve_strength > 0.0015f) adaptive_alpha = 0.4f;
        if (curve_strength > 0.003f)  adaptive_alpha = 0.2f;

        if (!left_coeff.empty()) {
            left_coeff = adaptive_alpha * prev_left_coeff_ + (1.0f - adaptive_alpha) * left_coeff;
            prev_left_coeff_ = left_coeff;
        } else {
            left_coeff = prev_left_coeff_;
        }
        if (!right_coeff.empty()) {
            right_coeff = adaptive_alpha * prev_right_coeff_ + (1.0f - adaptive_alpha) * right_coeff;
            prev_right_coeff_ = right_coeff;
        } else {
            right_coeff = prev_right_coeff_;
        }

        // --- DYNAMIC LANE WIDTH (rolling average) ---
        float lane_width_px = 120.0f;
        if (!left_coeff.empty() && !right_coeff.empty()) {
            float left_at_bottom = evaluatePoly(left_coeff, height - 10);
            float right_at_bottom = evaluatePoly(right_coeff, height - 10);
            lane_width_px = right_at_bottom - left_at_bottom;
            if (lane_width_px > 50 && lane_width_px < 300) {
                lane_width_history_.push_front(lane_width_px);
                if (lane_width_history_.size() > 10) lane_width_history_.pop_back();
                lane_width_px = std::accumulate(lane_width_history_.begin(), lane_width_history_.end(), 0.0f) / lane_width_history_.size();
            }
        }

        // --- GEOMETRIC VALIDATION ---
        bool width_ok = true;
        bool lanes_crossing = false;
        if (!left_coeff.empty() && !right_coeff.empty()) {
            // Check width at several y positions
            float w_sum = 0.0f;
            int count = 0;
            for (int y = height * 0.4; y < height; y += 30) {
                float lx = evaluatePoly(left_coeff, y);
                float rx = evaluatePoly(right_coeff, y);
                if (lx >= 0 && rx >= 0) {
                    w_sum += (rx - lx);
                    count++;
                }
                if (lx >= rx) lanes_crossing = true;
            }
            float avg_width = (count > 0) ? w_sum / count : 0.0f;
            width_ok = (avg_width > 60 && avg_width < 250);
        }

        // --- MULTI‑FACTOR CONFIDENCE ---
        float confidence = 1.0f;
        if (!left_coeff.empty() && !right_coeff.empty()) {
            confidence = 0.9f;  // base for both lanes
        } else if (!left_coeff.empty() || !right_coeff.empty()) {
            confidence = 0.5f;
        } else {
            confidence = 0.1f;
        }

        // Pixel count penalties
        if (left_pixels.size() < 100) confidence *= 0.7f;
        if (right_pixels.size() < 100) confidence *= 0.7f;

        // Width and crossing penalties
        if (!width_ok) confidence *= 0.5f;
        if (lanes_crossing) confidence *= 0.2f;

        // --- DYNAMIC LOOKAHEAD BASED ON CURVATURE ---
        float lookahead_ratio = 0.7f;   // default (far)
        if (curve_strength > 0.0015f) lookahead_ratio = 0.55f;
        if (curve_strength > 0.003f)  lookahead_ratio = 0.45f;
        float lookahead_y = height * lookahead_ratio;

        // --- TARGET LANE CENTER AT LOOKAHEAD ---
        float target_x = width / 2.0f;
        if (!left_coeff.empty() && !right_coeff.empty()) {
            float left_x = evaluatePoly(left_coeff, lookahead_y);
            float right_x = evaluatePoly(right_coeff, lookahead_y);
            target_x = (left_x + right_x) / 2.0f;
        } else if (!left_coeff.empty()) {
            float left_x = evaluatePoly(left_coeff, lookahead_y);
            target_x = left_x + lane_width_px / 2.0f;
        } else if (!right_coeff.empty()) {
            float right_x = evaluatePoly(right_coeff, lookahead_y);
            target_x = right_x - lane_width_px / 2.0f;
        } else {
            target_x = prev_lane_center_;
        }

        // Temporal smoothing of lane center
        target_x = 0.8f * prev_lane_center_ + 0.2f * target_x;
        prev_lane_center_ = target_x;

        // Steering error and command
        float error = (target_x - width/2.0f) / (width/2.0f);
        float steering = -error * 0.6f;   // P controller gain

        // Penalty for large steering (unstable detection)
        if (std::abs(steering) > 0.4f) confidence *= 0.8f;

        // --- SPEED CONTROL WITH CURVATURE & STEERING ---
        float max_speed = 0.5f;
        float speed = max_speed * std::max(0.2f, confidence);

        // Curve speed reduction
        float curve_speed_factor = 1.0f;
        if (curve_strength > 0.0008f) curve_speed_factor = 0.8f;
        if (curve_strength > 0.0015f) curve_speed_factor = 0.6f;
        if (curve_strength > 0.003f)  curve_speed_factor = 0.4f;
        speed *= curve_speed_factor;

        // Steering magnitude reduction
        float steer_mag = std::abs(steering);
        if (steer_mag > 0.25f) speed *= 0.7f;
        if (steer_mag > 0.45f) speed *= 0.5f;

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = std::clamp(speed, 0.05f, max_speed);
        cmd.angular.z = steering;
        cmd_vel_pub->publish(cmd);

        // --- DEBUG VISUALIZATION (unchanged) ---
        cv::Mat curve_display = original_image.clone();
        if (!left_coeff.empty()) {
            std::vector<cv::Point2f> bev_curve;
            for (int y = 0; y < height; y += 5) {
                float x = evaluatePoly(left_coeff, y);
                if (x >= 0 && x < width) bev_curve.push_back(cv::Point2f(x, y));
            }
            if (!bev_curve.empty()) {
                std::vector<cv::Point2f> orig_curve;
                cv::perspectiveTransform(bev_curve, orig_curve, inverse_transform);
                for (size_t i = 1; i < orig_curve.size(); ++i)
                    cv::line(curve_display, orig_curve[i-1], orig_curve[i], cv::Scalar(0, 0, 255), 5);
            }
        }
        if (!right_coeff.empty()) {
            std::vector<cv::Point2f> bev_curve;
            for (int y = 0; y < height; y += 5) {
                float x = evaluatePoly(right_coeff, y);
                if (x >= 0 && x < width) bev_curve.push_back(cv::Point2f(x, y));
            }
            if (!bev_curve.empty()) {
                std::vector<cv::Point2f> orig_curve;
                cv::perspectiveTransform(bev_curve, orig_curve, inverse_transform);
                for (size_t i = 1; i < orig_curve.size(); ++i)
                    cv::line(curve_display, orig_curve[i-1], orig_curve[i], cv::Scalar(255, 0, 0), 5);
            }
        }

        auto mask_msg = cv_bridge::CvImage(msg->header, "mono8", lane_mask).toImageMsg();
        debug_img_pub->publish(*mask_msg);
        auto bev_msg = cv_bridge::CvImage(msg->header, "mono8", birds_eye).toImageMsg();
        bev_pub->publish(*bev_msg);
        auto curve_msg = cv_bridge::CvImage(msg->header, "bgr8", curve_display).toImageMsg();
        curve_pub->publish(*curve_msg);

        static int frame_count = 0;
        if (frame_count++ % 30 == 0) {
            RCLCPP_INFO(this->get_logger(),
                        "Conf: %.2f | Curve: %.4f | Lookahead: %.1f | Speed: %.2f | Steering: %.3f",
                        confidence, curve_strength, lookahead_y, cmd.linear.x, steering);
        }
    }

    cv::Mat fitPolyFromPoints(const std::vector<cv::Point>& points, int degree) {
        if (points.size() < (size_t)degree + 1) return cv::Mat();
        std::vector<cv::Point2f> pts;
        for (auto& p : points) pts.push_back(cv::Point2f(p.x, p.y));
        std::sort(pts.begin(), pts.end(), [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });
        int n = pts.size();
        cv::Mat X(n, degree+1, CV_32F);
        cv::Mat Y(n, 1, CV_32F);
        for (int i = 0; i < n; ++i) {
            float y = pts[i].y;
            float x = pts[i].x;
            for (int j = 0; j <= degree; ++j)
                X.at<float>(i, j) = std::pow(y, j);
            Y.at<float>(i, 0) = x;
        }
        cv::Mat coeff;
        cv::solve(X, Y, coeff, cv::DECOMP_QR);
        return coeff;
    }

    float evaluatePoly(const cv::Mat& coeff, float y) {
        if (coeff.empty()) return -1;
        return coeff.at<float>(0) + coeff.at<float>(1)*y + coeff.at<float>(2)*y*y;
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_img_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr roi_display_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr bev_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr curve_pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;

    cv::Mat prev_left_coeff_, prev_right_coeff_;
    float prev_lane_center_;
    std::deque<float> lane_width_history_;
    int width_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaneDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
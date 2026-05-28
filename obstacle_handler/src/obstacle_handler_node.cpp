#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <deque>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

//  tunables 
static constexpr float CONF_THRESH       = 0.45f;
static constexpr float NMS_THRESH        = 0.40f;
static constexpr float DANGER_DIST_M     = 3.5f;   // trigger lane-change below this
static constexpr float CLEAR_DIST_M      = 5.5f;   // resume normal above this
static constexpr float LANE_CHANGE_STEER = 0.55f;  // angular.z magnitude during shift
static constexpr float LANE_CHANGE_SPEED = 0.25f;  // travel speed while shifting
static constexpr float HOLD_STEER_SEC    = 1.2f;   // how long to steer sideways
static constexpr float REALIGN_SEC       = 0.9f;   // counter-steer to straighten
// 

enum class State { NORMAL, SHIFTING, REALIGNING, CHANGED };

class ObstacleHandler : public rclcpp::Node {
public:
    ObstacleHandler()
    : Node("obstacle_handler"),
      nearest_dist_(9999.0f),
      shift_dir_(1.0f),
      state_(State::NORMAL),
      phase_start_(this->now())
    {
        //  parameters 
        this->declare_parameter("model_path",      "adas_sim/src/yolov8n.onnx");
        this->declare_parameter("input_width",      640);
        this->declare_parameter("input_height",     480);   // matches SDF: 640x480
        this->declare_parameter("focal_length_px",  381.0); // (640/2)/tan(40°) — SDF hfov=1.396rad(~80°)
        this->declare_parameter("path_zone_ratio",  0.30);  // ±% of width = "in our path"

        model_path_       = this->get_parameter("model_path").as_string();
        input_w_          = this->get_parameter("input_width").as_int();
        input_h_          = this->get_parameter("input_height").as_int();
        focal_len_        = (float)this->get_parameter("focal_length_px").as_double();
        path_zone_ratio_  = (float)this->get_parameter("path_zone_ratio").as_double();

        //  load YOLOv8 ONNX 
        try {
            net_ = cv::dnn::readNetFromONNX(model_path_);
            net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
            net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
            RCLCPP_INFO(this->get_logger(), "YOLO model loaded → %s", model_path_.c_str());
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Model load failed: %s", e.what());
        }

        //  ROS I/O 
        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&ObstacleHandler::image_cb, this, std::placeholders::_1));

        // lane detector must publish to /lane_cmd_vel (not /cmd_vel directly)
        // this node arbitrates and forwards the final result to /cmd_vel
        // which the Ackermann plugin in the SDF is already listening to
        lane_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/lane_cmd_vel", 10,
            std::bind(&ObstacleHandler::lane_cmd_cb, this, std::placeholders::_1));

        // publishes directly to /cmd_vel — Ackermann plugin topic in SDF
        final_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        debug_pub_     = this->create_publisher<sensor_msgs::msg::Image>("/obstacle_debug", 10);

        RCLCPP_INFO(this->get_logger(), "ObstacleHandler ready. Danger threshold: %.1f m", DANGER_DIST_M);
    }

private:
    // 
    void image_cb(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
        last_header_  = msg->header;

        if (net_.empty()) return;

        //  inference 
        // YOLOv8 always wants a square blob — pad/scale to 640×640
        // then map detections back to actual frame dimensions
        cv::Mat blob;
        cv::dnn::blobFromImage(frame, blob, 1.0 / 255.0,
                               cv::Size(640, 640),
                               cv::Scalar(), /*swapRB=*/true, false);
        net_.setInput(blob);

        std::vector<cv::Mat> raw_out;
        net_.forward(raw_out, net_.getUnconnectedOutLayersNames());

        // YOLOv8 ONNX output tensor: [1, 84, 8400]
        // reshape to [8400, 84] — each row is one detection proposal
        cv::Mat preds = raw_out[0].reshape(1, raw_out[0].size[1]).t();

        int frame_w = frame.cols;   // 640 (from SDF)
        int frame_h = frame.rows;   // 480 (from SDF)
        // scale factors: bbox coords are normalised to 640×640 blob
        float sx = (float)frame_w / 640.0f;
        float sy = (float)frame_h / 640.0f;

        std::vector<cv::Rect2d> boxes;
        std::vector<float>      confs;
        std::vector<int>        cls_ids;

        for (int i = 0; i < preds.rows; ++i) {
            float* row = preds.ptr<float>(i);

            // find best class score across 80 COCO classes
            cv::Mat class_scores(1, 80, CV_32F, row + 4);
            cv::Point max_loc;
            double    max_val;
            cv::minMaxLoc(class_scores, nullptr, &max_val, nullptr, &max_loc);

            if (max_val < CONF_THRESH) continue;

            // bbox is cx, cy, w, h — in 640×640 blob space, scale to frame
            double cx = row[0] * sx;
            double cy = row[1] * sy;
            double bw = row[2] * sx;
            double bh = row[3] * sy;

            boxes.push_back(cv::Rect2d(cx - bw/2, cy - bh/2, bw, bh));
            confs.push_back((float)max_val);
            cls_ids.push_back(max_loc.x);
        }

        std::vector<int> keep;
        cv::dnn::NMSBoxes(boxes, confs, CONF_THRESH, NMS_THRESH, keep);

        //  find closest threat in our path 
        float closest_dist = 9999.0f;
        int   closest_idx  = -1;
        int   center_x     = frame_w / 2;

        for (int idx : keep) {
            cv::Rect2d& b  = boxes[idx];
            int box_cx = (int)(b.x + b.width / 2.0);

            // skip detections that are clearly off to the side
            if (std::abs(box_cx - center_x) > frame_w * path_zone_ratio_) continue;

            // monocular depth: dist = (real_height_m × focal_length_px) / pixel_height
            float real_h  = real_height_m(cls_ids[idx]);
            float dist    = (real_h * focal_len_) / (float)b.height;

            if (dist < closest_dist) {
                closest_dist = dist;
                closest_idx  = idx;
            }
        }

        nearest_dist_ = closest_dist;

        // obstacle left of centre → shift right (+1), obstacle right → shift left (−1)
        if (closest_idx >= 0) {
            int ob_cx = (int)(boxes[closest_idx].x + boxes[closest_idx].width / 2.0);
            shift_dir_ = (ob_cx < center_x) ? 1.0f : -1.0f;
        }

        //  annotate debug image 
        cv::Mat dbg = frame.clone();
        for (int idx : keep) {
            cv::Rect2d& b      = boxes[idx];
            bool        threat = (idx == closest_idx && closest_dist < DANGER_DIST_M);
            cv::Scalar  col    = threat ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 200, 60);
            cv::rectangle(dbg, b, col, 2);

            float dist_label = (real_height_m(cls_ids[idx]) * focal_len_) / (float)b.height;
            cv::putText(dbg,
                        cv::format("%.1fm", dist_label),
                        cv::Point((int)b.x, (int)b.y - 6),
                        cv::FONT_HERSHEY_SIMPLEX, 0.55, col, 2);
        }
        cv::putText(dbg, "State: " + state_str(),
                    cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255,255,0), 2);
        cv::putText(dbg, cv::format("Nearest: %.2f m", nearest_dist_),
                    cv::Point(10, 58), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255,255,0), 2);

        debug_pub_->publish(*cv_bridge::CvImage(last_header_, "bgr8", dbg).toImageMsg());
    }

    // 
    void lane_cmd_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        rclcpp::Time now     = this->now();
        double       elapsed = (now - phase_start_).seconds();

        switch (state_) {

        //  steady state: pass through lane-detector command 
        case State::NORMAL:
            final_cmd_pub_->publish(*msg);
            if (nearest_dist_ < DANGER_DIST_M) {
                state_       = State::SHIFTING;
                phase_start_ = now;
                RCLCPP_WARN(this->get_logger(),
                    "Obstacle at %.2f m — lane change triggered (shift_dir=%.0f)",
                    nearest_dist_, shift_dir_);
            }
            break;

        //  phase 1: steer hard into adjacent lane 
        case State::SHIFTING: {
            geometry_msgs::msg::Twist cmd;
            cmd.linear.x  = LANE_CHANGE_SPEED;
            cmd.angular.z = shift_dir_ * LANE_CHANGE_STEER;
            final_cmd_pub_->publish(cmd);

            if (elapsed > HOLD_STEER_SEC) {
                state_       = State::REALIGNING;
                phase_start_ = now;
                RCLCPP_INFO(this->get_logger(), "Shift done — realigning");
            }
            break;
        }

        //  phase 2: counter-steer to parallel the new lane 
        case State::REALIGNING: {
            geometry_msgs::msg::Twist cmd;
            cmd.linear.x  = LANE_CHANGE_SPEED;
            // slightly weaker counter-steer so we don't overshoot back
            cmd.angular.z = -shift_dir_ * LANE_CHANGE_STEER * 0.75f;
            final_cmd_pub_->publish(cmd);

            if (elapsed > REALIGN_SEC) {
                state_       = State::CHANGED;
                phase_start_ = now;
                RCLCPP_INFO(this->get_logger(), "Lane change complete");
            }
            break;
        }

        //  phase 3: hand back to lane detector; watch for clearance 
        case State::CHANGED:
            final_cmd_pub_->publish(*msg);
            if (nearest_dist_ > CLEAR_DIST_M || elapsed > 6.0) {
                state_ = State::NORMAL;
                RCLCPP_INFO(this->get_logger(), "Obstacle cleared — resuming normal follow");
            }
            break;
        }
    }

    // 
    // Approximate real-world heights for COCO class IDs.
    // Used for monocular depth estimation via similar-triangles formula.
    float real_height_m(int cls_id) {
        switch (cls_id) {
            case 0:  return 1.75f;  // person
            case 1:  return 1.00f;  // bicycle
            case 2:  return 1.50f;  // car
            case 3:  return 1.20f;  // motorbike
            case 5:  return 3.50f;  // bus
            case 7:  return 4.00f;  // truck
            case 9:  return 0.90f;  // traffic light
            case 11: return 0.80f;  // stop sign
            default: return 1.20f;  // generic fallback
        }
    }

    std::string state_str() {
        switch (state_) {
            case State::NORMAL:     return "NORMAL";
            case State::SHIFTING:   return "SHIFTING";
            case State::REALIGNING: return "REALIGNING";
            case State::CHANGED:    return "CHANGED";
        }
        return "UNKNOWN";
    }

    //  members 
    cv::dnn::Net net_;
    std::string  model_path_;
    int          input_w_, input_h_;
    float        focal_len_;
    float        path_zone_ratio_;

    std_msgs::msg::Header last_header_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr   img_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr lane_cmd_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr    final_cmd_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr      debug_pub_;

    // order here must match constructor initialiser list to silence -Wreorder
    float        nearest_dist_;
    float        shift_dir_;
    State        state_;
    rclcpp::Time phase_start_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleHandler>());
    rclcpp::shutdown();
    return 0;
}
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"

#if __has_include("cv_bridge/cv_bridge.hpp")
#include "cv_bridge/cv_bridge.hpp"
#else
#include "cv_bridge/cv_bridge.h"
#endif
#include "opencv2/opencv.hpp"

// Task 4:
// Detect green plastic caps in the camera image.
// This node also acts as a small supervisor:
// - if no cap is visible, it forwards the navigation command from Task1;
// - if a cap is visible, it overrides Task1 and visually servo-controls the robot;
// - Task3 can still filter the final command for obstacle avoidance.
class CameraCapsNode : public rclcpp::Node
{
public:
  CameraCapsNode() : Node("task4_camera_caps")
  {
    // Camera and command topics are parameters so the node works in sim and on the robot.
    image_topic_ = declare_parameter("image_topic", "/camera/image_raw");
    input_cmd_vel_topic_ = declare_parameter("input_cmd_vel_topic", "/cmd_vel_nav");
    output_cmd_vel_topic_ = declare_parameter("output_cmd_vel_topic", "/cmd_vel");
    use_stamped_cmd_vel_ = declare_parameter("use_stamped_cmd_vel", true);

    // If disabled, the node only reports detection and does not move the robot.
    enable_visual_servo_ = declare_parameter("enable_visual_servo", true);

    // Detection and approach thresholds. Area grows when the cap gets closer.
    min_area_ = declare_parameter("min_area", 300.0);
    stop_area_ = declare_parameter("stop_area", 6000.0);
    approach_speed_ = declare_parameter("approach_speed", 0.08);
    angular_gain_ = declare_parameter("angular_gain", 0.002);

    detected_pub_ = create_publisher<std_msgs::msg::Bool>("/cap_detected", 10);

    // Use TwistStamped by default because the Jazzy Gazebo bridge expects it.
    if (use_stamped_cmd_vel_) {
      stamped_cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(output_cmd_vel_topic_, 10);
      stamped_cmd_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        input_cmd_vel_topic_,
        10,
        std::bind(&CameraCapsNode::stampedCmdCallback, this, std::placeholders::_1));
    } else {
      cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_cmd_vel_topic_, 10);
      cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        input_cmd_vel_topic_,
        10,
        std::bind(&CameraCapsNode::cmdCallback, this, std::placeholders::_1));
    }

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_,
      10,
      std::bind(&CameraCapsNode::imageCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(
      get_logger(),
      "Task4 camera caps started on %s, supervising %s -> %s, cmd_vel type=%s, visual_servo=%s",
      image_topic_.c_str(),
      input_cmd_vel_topic_.c_str(),
      output_cmd_vel_topic_.c_str(),
      use_stamped_cmd_vel_ ? "TwistStamped" : "Twist",
      enable_visual_servo_ ? "true" : "false");
  }

private:
  void stampedCmdCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    last_stamped_nav_cmd_ = *msg;
    nav_cmd_received_ = true;
  }

  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_nav_cmd_ = *msg;
    nav_cmd_received_ = true;
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv::Mat frame;

    try {
      // Convert ROS Image to OpenCV BGR image.
      frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
      return;
    }

    cv::Mat hsv;
    // HSV separates color from brightness better than raw BGR.
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    cv::Mat mask1;
    cv::Mat mask2;
    cv::Mat mask;

    // Green cap segmentation. Tune these HSV bounds if lighting changes.
    cv::inRange(
      hsv,
      cv::Scalar(35, 70, 70),
      cv::Scalar(85, 255, 255),
      mask
    );

    // Remove small isolated pixels, then reconnect the main cap blob.
    cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 1);
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

    // Contours are connected green regions. The largest one is assumed to be the cap.
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(
      mask,
      contours,
      cv::RETR_EXTERNAL,
      cv::CHAIN_APPROX_SIMPLE
    );

    bool cap_detected = false;
    double area = 0.0;
    double error = 0.0;

    if (!contours.empty()) {
      auto largest_it = std::max_element(
        contours.begin(),
        contours.end(),
        [](const std::vector<cv::Point> & a, const std::vector<cv::Point> & b) {
          return cv::contourArea(a) < cv::contourArea(b);
        }
      );

      area = cv::contourArea(*largest_it);

      if (area > min_area_) {
        cv::Rect box = cv::boundingRect(*largest_it);

        // error < 0 means cap is left of image center, error > 0 means right.
        double center_x = box.x + box.width / 2.0;
        double image_center = frame.cols / 2.0;
        error = center_x - image_center;
        cap_detected = true;

        RCLCPP_INFO_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          1000,
          "Cap detected: area=%.1f, error=%.1f",
          area,
          error
        );
      }
    }

    // Boolean topic useful in rqt/rviz/terminal to prove Task4 sees a cap.
    std_msgs::msg::Bool detected_msg;
    detected_msg.data = cap_detected;
    detected_pub_->publish(detected_msg);

    if (!enable_visual_servo_) {
      forwardNavigationCommand();
      return;
    }

    // Visual servo: approach the cap while steering to keep it centered.
    if (cap_detected) {
      const double linear_x = area >= stop_area_ ? 0.0 : approach_speed_;
      publishVelocity(linear_x, -angular_gain_ * error);
      return;
    }

    // No cap visible: keep Task1 in charge if it is providing a navigation command.
    if (forwardNavigationCommand()) {
      return;
    }

    // If Task4 is launched alone or Task1 is not publishing yet, do not invent a path.
    // The supervisor only forwards Task1 or overrides it when a cap is visible.
    publishVelocity(0.0, 0.0);
  }

  bool forwardNavigationCommand()
  {
    if (!nav_cmd_received_) {
      return false;
    }

    if (use_stamped_cmd_vel_) {
      last_stamped_nav_cmd_.header.stamp = now();
      stamped_cmd_pub_->publish(last_stamped_nav_cmd_);
    } else {
      cmd_pub_->publish(last_nav_cmd_);
    }

    return true;
  }

  void publishVelocity(double linear_x, double angular_z)
  {
    // Publish the selected velocity format.
    if (use_stamped_cmd_vel_) {
      geometry_msgs::msg::TwistStamped cmd;
      cmd.header.stamp = now();
      cmd.twist.linear.x = linear_x;
      cmd.twist.angular.z = angular_z;
      stamped_cmd_pub_->publish(cmd);
      return;
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear_x;
    cmd.angular.z = angular_z;
    cmd_pub_->publish(cmd);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr stamped_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr detected_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr stamped_cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

  std::string image_topic_;
  std::string input_cmd_vel_topic_;
  std::string output_cmd_vel_topic_;
  bool use_stamped_cmd_vel_ = true;
  bool enable_visual_servo_ = true;
  double min_area_ = 300.0;
  double stop_area_ = 6000.0;
  double approach_speed_ = 0.08;
  double angular_gain_ = 0.002;
  bool nav_cmd_received_ = false;
  geometry_msgs::msg::Twist last_nav_cmd_;
  geometry_msgs::msg::TwistStamped last_stamped_nav_cmd_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraCapsNode>());
  rclcpp::shutdown();
  return 0;
}

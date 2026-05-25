#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::placeholders;
using namespace std::chrono_literals;

class OdomNode : public rclcpp::Node
{
public:
    OdomNode() : Node("turtlebot_odom")
    {
        joints_subscriber_ = create_subscription<sensor_msgs::msg::JointState>("joint_states", 10,
                                            std::bind(&OdomNode::callbackOdomSub, this, _1));
        odom_timer_ = create_wall_timer(0.5s, std::bind(&OdomNode::callbackOdomTimer, this));
        odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("turtlebot_odom", 10);


    }

    void callbackOdomSub(const sensor_msgs::msg::JointState::SharedPtr msg){
        wheels_current_pos_ = msg->position;
        if(update_init_pose_){
            wheels_init_pos_ = wheels_current_pos_;
            update_init_pose_ = false;
        }

    }

    void callbackOdomTimer(){

        if (!wheels_current_pos_.empty() & !wheels_init_pos_.empty()){
            double cur_left_pos = wheels_current_pos_.at(0);
            double cur_right_pos = wheels_current_pos_.at(1);
            double init_left_pos = wheels_init_pos_.at(0);
            double init_right_pos = wheels_init_pos_.at(1);          
            
        
            double delta_ql = cur_left_pos - init_left_pos;
            double delta_qr = cur_right_pos - init_right_pos;
            double delta_D = wheel_rad_*(delta_qr + delta_ql)/2;
            double delta_theta = wheel_rad_*(delta_qr - delta_ql)/track_width_;
            x_ = x_ + delta_D*cos(theta_ + delta_theta/2);
            y_ = y_ + delta_D*sin(theta_ + delta_theta/2);
            theta_ = theta_ + delta_theta;
            
            // udpate the initial pose
            update_init_pose_ = true;
            auto msg = nav_msgs::msg::Odometry();

            //convert theta to a quaternion
            tf2::Quaternion tf2_quat;
            tf2_quat.setRPY(0, 0, theta_);
            tf2_quat.normalize();
            geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(tf2_quat);

            msg.header.frame_id = "turtlebot_odom";
            msg.header.stamp = this->get_clock()->now();
            msg.pose.pose.position.x = x_;
            msg.pose.pose.position.y = y_;
            msg.pose.pose.orientation = msg_quat;

            odom_publisher_->publish(msg);
        }

    }

private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::TimerBase::SharedPtr odom_timer_;
    std::vector<double> wheels_current_pos_;
    std::vector<double> wheels_init_pos_;
    double x_ = 0;
    double y_ = 0;
    double theta_ = 0;
    bool update_init_pose_ = true;
    double wheel_rad_ = 0.066/2; // in meters
    double track_width_ = 0.287; // in meters
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
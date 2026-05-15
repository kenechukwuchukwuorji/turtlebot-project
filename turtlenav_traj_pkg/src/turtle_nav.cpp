#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <algorithm>

using namespace std::chrono_literals;

namespace my_pkg
{

    class MyNode : public rclcpp::Node
    {
    public:
        MyNode(rclcpp::NodeOptions options) : Node("my_node", options)
        {
            // init subscribers
            pose_subscription = create_subscription<nav_msgs::msg::Odometry>("/odom", 10, // subscribe kaycee's topic
                                                                          [this](nav_msgs::msg::Odometry::SharedPtr msg)
                                                                          {
                                                                              input_msg = *msg;
                                                                          });

            
            // init publishers

            velocity_publisher = create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
            // topic + QoS

            // init timer - the function will be called with the given rate
            publish_timer = create_wall_timer(100ms, // rate
                                              [&]()
                                              { callback_time(); });

            goal_x_ = 50.0;
            goal_y_ = 5.0;
            
            // PID GAINS Linear and Angular
            Kp_lin = 1.0;
            Kp_ang = 1.0;
        }

    private:
        // declare any subscriber / publisher / timer
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_subscription;
        
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_publisher;

        nav_msgs::msg::Odometry input_msg;
        geometry_msgs::msg::PoseStamped goal_input_msg;

        rclcpp::TimerBase::SharedPtr publish_timer;

        double goal_x_, goal_y_, Kp_lin, Kp_ang;

        void callback_time()
        {
            
            geometry_msgs::msg::TwistStamped vel_msg;
            
            double curr_x = input_msg.pose.pose.position.x;
            double curr_y = input_msg.pose.pose.position.y;

            
            double q_z = input_msg.pose.pose.orientation.z;
            double q_w = input_msg.pose.pose.orientation.w;
            
            double curr_yaw = 2.0 * std::atan2(q_z, q_w);

        
            double dist_err = calculate_distance();
            double angle_to_goal = std::atan2(goal_y_ - curr_y, goal_x_ - curr_x);
            
            
            double angle_error = angle_to_goal - curr_yaw;
            
            // Normalize to [-pi, pi] using atan2(sin, cos) - very robust
            angle_error = std::atan2(std::sin(angle_error), std::cos(angle_error));

            
            if (dist_err < 0.1) {
                vel_msg.twist.linear.x = 0.0;
                vel_msg.twist.angular.z = 0.0;
                RCLCPP_INFO(this->get_logger(), "Goal Reached!");
            } else {
            
                vel_msg.twist.linear.x = std::min(0.22, Kp_lin * dist_err); 
                vel_msg.twist.angular.z = Kp_ang * angle_error;
            }
        
            velocity_publisher->publish(vel_msg);
        }


// functions

        double calculate_distance()
        {
            return std::sqrt(std::pow((goal_x_ - input_msg.pose.pose.position.x), 2) + std::pow((goal_y_ - input_msg.pose.pose.position.y), 2));      
        }
    };

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    std::cout << "welcome to hell" << std::endl;
    rclcpp::spin(std::make_shared<my_pkg::MyNode>(options));
    rclcpp::shutdown();
    return 0;
}
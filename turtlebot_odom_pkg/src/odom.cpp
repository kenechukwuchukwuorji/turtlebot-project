#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

class OdomNode : public rclcpp::Node
{
public:
    OdomNode() : Node("turtlebot_odom")
    {

    }

private:
    rclcpp::Publisher
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
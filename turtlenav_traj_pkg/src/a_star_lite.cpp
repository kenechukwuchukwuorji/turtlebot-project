/**
 * @file astar_node.cpp
 * @brief Lightweight, standalone A* path planner for 2D Grid Maps
 */

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <vector>
#include <queue>
#include <cmath>
#include <memory>
#include <map>

struct GridPoint {
    int x, y;
    bool operator==(const GridPoint& o) const { return x == o.x && y == o.y; }
    bool operator<(const GridPoint& o) const { return std::tie(x, y) < std::tie(o.x, o.y); }
};

struct AStarNode {
    GridPoint pt;
    double g, h, f;
    std::shared_ptr<AStarNode> parent;
};

// Comparator for priority queue (lowest F score first)
struct CompareF {
    bool operator()(const std::shared_ptr<AStarNode>& a, const std::shared_ptr<AStarNode>& b) {
        return a->f > b->f;
    }
};

class SimpleAStar : public rclcpp::Node {
public:
    SimpleAStar() : Node("astar_node") {
        // Match the Transient Local QoS profile your controller uses for /map
        auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
        
        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", map_qos, std::bind(&SimpleAStar::mapCallback, this, std::placeholders::_1));

        start_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "turtlebot_odom", 10, 
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                start_pose_ = msg->pose.pose;
                has_start_ = true;
                planPath(); // Updates and re-plans the dynamic path as the robot drives
            });

        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&SimpleAStar::goalCallback, this, std::placeholders::_1));

        path_pub_ = create_publisher<nav_msgs::msg::Path>("/astar/nav_path", 10);

        RCLCPP_INFO(this->get_logger(), "Custom Lightweight A* Engine Initialized!");
    }

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr start_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    nav_msgs::msg::OccupancyGrid current_map_;
    geometry_msgs::msg::Pose start_pose_;
    geometry_msgs::msg::Pose goal_pose_;

    bool has_map_ = false;
    bool has_start_ = false;
    bool has_goal_ = false;

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        current_map_ = *msg;
        has_map_ = true;
        planPath(); // Re-plan dynamically whenever the map layout updates!
    }

    void startCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        start_pose_ = msg->pose.pose;
        has_start_ = true;
    }

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        goal_pose_ = msg->pose;
        has_goal_ = true;
        planPath();
    }

    GridPoint worldToGrid(double wx, double wy) {
        GridPoint pt;
        pt.x = static_cast<int>((wx - current_map_.info.origin.position.x) / current_map_.info.resolution);
        pt.y = static_cast<int>((wy - current_map_.info.origin.position.y) / current_map_.info.resolution);
        return pt;
    }

    void gridToWorld(int gx, int gy, double& wx, double& wy) {
        wx = current_map_.info.origin.position.x + (gx + 0.5) * current_map_.info.resolution;
        wy = current_map_.info.origin.position.y + (gy + 0.5) * current_map_.info.resolution;
    }

    void planPath() {
        if (!has_map_ || !has_start_ || !has_goal_) return;

        GridPoint start = worldToGrid(start_pose_.position.x, start_pose_.position.y);
        GridPoint goal = worldToGrid(goal_pose_.position.x, goal_pose_.position.y);

        // Sanity check map boundaries
        if (start.x < 0 || start.x >= (int)current_map_.info.width || start.y < 0 || start.y >= (int)current_map_.info.height ||
            goal.x < 0 || goal.x >= (int)current_map_.info.width || goal.y < 0 || goal.y >= (int)current_map_.info.height) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Start or Goal is outside current local map bounds.");
            return;
        }

        // Standard A* Solver Logic
        std::priority_queue<std::shared_ptr<AStarNode>, std::vector<std::shared_ptr<AStarNode>>, CompareF> open_list;
        std::map<GridPoint, double> closed_list; 

        auto start_node = std::make_shared<AStarNode>();
        start_node->pt = start;
        start_node->g = 0.0;
        start_node->h = std::hypot(start.x - goal.x, start.y - goal.y);
        start_node->f = start_node->g + start_node->h;
        start_node->parent = nullptr;

        open_list.push(start_node);

        std::shared_ptr<AStarNode> current = nullptr;
        bool found_path = false;

        int dx[] = {-1, 1, 0, 0, -1, -1, 1, 1};
        int dy[] = {0, 0, -1, 1, -1, 1, -1, 1};

        while (!open_list.empty()) {
            current = open_list.top();
            open_list.pop();

            if (current->pt == goal) {
                found_path = true;
                break;
            }

            closed_list[current->pt] = current->g;

            for (int i = 0; i < 8; ++i) {
                GridPoint next_pt = {current->pt.x + dx[i], current->pt.y + dy[i]};

                if (next_pt.x < 0 || next_pt.x >= (int)current_map_.info.width || 
                    next_pt.y < 0 || next_pt.y >= (int)current_map_.info.height) continue;

                // Value 100 means an obstacle is occupied
                if (current_map_.data[next_pt.y * current_map_.info.width + next_pt.x] >= 50) continue;

                double movement_cost = (dx[i] != 0 && dy[i] != 0) ? 1.414 : 1.0;
                double next_g = current->g + movement_cost;

                if (closed_list.find(next_pt) != closed_list.end() && next_g >= closed_list[next_pt]) continue;

                auto neighbor = std::make_shared<AStarNode>();
                neighbor->pt = next_pt;
                neighbor->g = next_g;
                neighbor->h = std::hypot(next_pt.x - goal.x, next_pt.y - goal.y);
                neighbor->f = neighbor->g + neighbor->h;
                neighbor->parent = current;

                open_list.push(neighbor);
            }
        }

        if (found_path) {
            nav_msgs::msg::Path path_msg;
            path_msg.header.stamp = this->get_clock()->now();
            path_msg.header.frame_id = "odom";

            std::vector<geometry_msgs::msg::PoseStamped> poses;
            while (current != nullptr) {
                geometry_msgs::msg::PoseStamped p;
                p.header.frame_id = "odom";
                gridToWorld(current->pt.x, current->pt.y, p.pose.position.x, p.pose.position.y);
                p.pose.position.z = 0.0;
                poses.push_back(p);
                current = current->parent;
            }
            // Reverse path to flow from Start -> Goal
            std::reverse(poses.begin(), poses.end());
            path_msg.poses = poses;
            path_pub_->publish(path_msg);
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::make_shared<SimpleAStar>();
    rclcpp::spin(std::make_shared<SimpleAStar>());
    rclcpp::shutdown();
    return 0;
}
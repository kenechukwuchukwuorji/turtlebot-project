#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp" 
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include <algorithm>
#include <cmath>
#include <vector>
#include <queue>
#include <deque> // Perfect for a First-In-First-Out (FIFO) queue
#include <map>
#include <memory>
#include <iostream>

using namespace std::chrono_literals;

// Lightweight stack structures for the optimized A* engine (No pointers)
struct GridPoint {
    int x, y;
    bool operator==(const GridPoint& o) const { return x == o.x && y == o.y; }
    bool operator<(const GridPoint& o) const { return std::tie(x, y) < std::tie(o.x, o.y); }
};

struct AStarNode {
    GridPoint pt;
    double g, h, f;
    GridPoint parent_pt; 
};

namespace turtle_nav
{
    class MyNode : public rclcpp::Node
    {
    public:
        MyNode(rclcpp::NodeOptions options) : Node("turtle_nav_node", options)
        {
            // 1. Setup the abstraction parameter
            this->declare_parameter<bool>("use_stamped_cmd_vel", false);
            use_stamped_cmd_vel_ = this->get_parameter("use_stamped_cmd_vel").as_bool();

            if (use_stamped_cmd_vel_) {
                RCLCPP_INFO(this->get_logger(), "Publishing TwistStamped (Simulation Mode)");
                cmd_vel_stamped_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);
            } else {
                RCLCPP_INFO(this->get_logger(), "Publishing Twist (Physical Robot Mode)");
                cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
            }

            // 2. Initialize Subscribers
            pose_subscription = create_subscription<nav_msgs::msg::Odometry>(
                "odom", 10, [this](nav_msgs::msg::Odometry::SharedPtr msg) {
                    input_msg = *msg;
                    has_odom = true;
                });

            scan_subscription = create_subscription<sensor_msgs::msg::LaserScan>(
                "scan", 10, [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
                    input_scan = *msg;
                    has_scan = true;
                });

            goal_subscription = create_subscription<geometry_msgs::msg::PoseStamped>(
                "/goal_pose", 10, [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                    global_goal_x = msg->pose.position.x;
                    global_goal_y = msg->pose.position.y;
                    has_goal = true;
                    RCLCPP_INFO(this->get_logger(), "Goal locked in at world coordinates: X: %.2f, Y: %.2f", global_goal_x, global_goal_y);
                    execute_astar_planning();
                });

            waypoint_sub_ = create_subscription<geometry_msgs::msg::PoseArray>("/map_grid",10, std::bind(&MyNode::waypointCallback, this, std::placeholders::_1));   

            // 3. Initialize RViz Publishers
            path_publisher = create_publisher<nav_msgs::msg::Path>("/astar/nav_path", 10);
            auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
            map_publisher = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", map_qos);

            // 4. Timers
            publish_timer = create_wall_timer(100ms, [this]() { callback_control_loop(); });
            map_publish_timer = create_wall_timer(500ms, [this]() { publish_map_and_replan(); });                                  
            
            // 5. Default Variable Setup
            map_width = 80;                     
            map_height = 80;                    
            map_resolution = 0.05;              
            obstacle_distance_threshold = 1.5;  
            
            Kp_lin = 0.6;                       
            Kp_ang = 1.4;                       

            has_odom = false;
            has_scan = false;
            has_goal = false;
            has_path = false;
            
            global_goal_x = 999.0;
            global_goal_y = 999.0;
            current_waypoint_idx = 0;
            last_path_update_time_ = this->get_clock()->now();
        }

    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_subscription;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription;
        rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr waypoint_sub_;
        
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_stamped_pub_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher;

        sensor_msgs::msg::LaserScan input_scan; 
        nav_msgs::msg::Odometry input_msg;
        nav_msgs::msg::OccupancyGrid map_msg;
        nav_msgs::msg::Path calculated_path;

        rclcpp::TimerBase::SharedPtr publish_timer;
        rclcpp::TimerBase::SharedPtr map_publish_timer;
        rclcpp::Time last_path_update_time_;

        double global_goal_x, global_goal_y, Kp_lin, Kp_ang;
        size_t current_waypoint_idx;
        
        int map_width, map_height;
        double map_resolution, obstacle_distance_threshold;
        bool has_odom, has_scan, has_goal, has_path, use_stamped_cmd_vel_;
        bool mission_complete_ = false;

        std::deque<std::pair<double, double>> waypoint_queue_;

        GridPoint worldToGrid(double wx, double wy) {
            GridPoint pt;
            pt.x = static_cast<int>((wx - map_msg.info.origin.position.x) / map_resolution);
            pt.y = static_cast<int>((wy - map_msg.info.origin.position.y) / map_resolution);
            return pt;
        }

        void gridToWorld(int gx, int gy, double& wx, double& wy) {
            wx = map_msg.info.origin.position.x + (gx + 0.5) * map_resolution;
            wy = map_msg.info.origin.position.y + (gy + 0.5) * map_resolution;
        }

        // Helper function to handle the dynamic velocity publishing
        void publish_velocity(double linear_x, double angular_z) {
            if (use_stamped_cmd_vel_) {
                geometry_msgs::msg::TwistStamped msg;
                msg.header.stamp = this->get_clock()->now();
                msg.header.frame_id = "base_link";
                msg.twist.linear.x = linear_x;
                msg.twist.angular.z = angular_z;
                cmd_vel_stamped_pub_->publish(msg);
            } else {
                geometry_msgs::msg::Twist msg;
                msg.linear.x = linear_x;
                msg.angular.z = angular_z;
                cmd_vel_pub_->publish(msg);
            }
        }

        void waypointCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
        {
        waypoint_queue_.clear(); // Clear any old missions

        for (const auto& pose : msg->poses) {
            waypoint_queue_.push_back({pose.position.x, pose.position.y});
        }

        RCLCPP_INFO(this->get_logger(), "Received %zu waypoints!", waypoint_queue_.size());
        mission_complete_ = false;
        
        // Instantly load the very first waypoint
        loadNextWaypoint();
        }

        void loadNextWaypoint()
        {
        if (waypoint_queue_.empty()) {
            RCLCPP_INFO(this->get_logger(), "All waypoints reached. Mission Complete!");
            mission_complete_ = true;
            return;
        }

        // Grab the front waypoint and remove it from the list
        auto next_target = waypoint_queue_.front();
        waypoint_queue_.pop_front();

        // Overwrite your existing A* global goal variables
        global_goal_x = next_target.first;
        global_goal_y = next_target.second;

        RCLCPP_INFO(this->get_logger(), "Heading to new waypoint: [%.2f, %.2f]", global_goal_x, global_goal_y);
        }

        void publish_map_and_replan()
        {
            if (!has_scan || !has_odom) return;

            map_msg.header.stamp = this->get_clock()->now();
            map_msg.header.frame_id = "odom"; 
            map_msg.info.resolution = map_resolution;
            map_msg.info.width = map_width;
            map_msg.info.height = map_height;
            
            map_msg.info.origin.position.x = input_msg.pose.pose.position.x - 2.0;
            map_msg.info.origin.position.y = input_msg.pose.pose.position.y - 2.0;
            map_msg.info.origin.position.z = 0.0;
            map_msg.info.origin.orientation.w = 1.0;

            map_msg.data.assign(static_cast<size_t>(map_width * map_height), 0);

            double curr_x = input_msg.pose.pose.position.x;
            double curr_y = input_msg.pose.pose.position.y;
            
            double q_x = input_msg.pose.pose.orientation.x;
            double q_y = input_msg.pose.pose.orientation.y;
            double q_z = input_msg.pose.pose.orientation.z;
            double q_w = input_msg.pose.pose.orientation.w;
            double curr_yaw = std::atan2(2.0 * (q_w * q_z + q_x * q_y), 1.0 - 2.0 * (q_y * q_y + q_z * q_z));

            int inflation_radius = 5; 

            for (size_t i = 0; i < input_scan.ranges.size(); ++i) {
                double range_val = input_scan.ranges[i];
                if (std::isnan(range_val) || std::isinf(range_val) || range_val < input_scan.range_min || range_val > obstacle_distance_threshold) continue;

                double local_angle = input_scan.angle_min + (static_cast<double>(i) * input_scan.angle_increment);
                double global_angle = curr_yaw + local_angle;
                
                double x_global = curr_x + (range_val * std::cos(global_angle));
                double y_global = curr_y + (range_val * std::sin(global_angle));

                GridPoint gp = worldToGrid(x_global, y_global);
                if (gp.x >= 0 && gp.x < map_width && gp.y >= 0 && gp.y < map_height) {
                    for (int dx = -inflation_radius; dx <= inflation_radius; ++dx) {
                        for (int dy = -inflation_radius; dy <= inflation_radius; ++dy) {
                            if (std::hypot(dx, dy) <= inflation_radius) {
                                int nx = gp.x + dx; 
                                int ny = gp.y + dy;
                                if (nx >= 0 && nx < map_width && ny >= 0 && ny < map_height) {
                                    map_msg.data[static_cast<size_t>(ny * map_width + nx)] = 100;
                                }
                            }
                        }
                    }
                }
            }

            GridPoint robot_center = worldToGrid(curr_x, curr_y);
            int clear_radius = 5; 
            for (int cx = -clear_radius; cx <= clear_radius; ++cx) {
                for (int cy = -clear_radius; cy <= clear_radius; ++cy) {
                    int nx = robot_center.x + cx; 
                    int ny = robot_center.y + cy;
                    if (nx >= 0 && nx < map_width && ny >= 0 && ny < map_height) {
                        map_msg.data[static_cast<size_t>(ny * map_width + nx)] = 0;
                    }
                }
            }

            map_publisher->publish(map_msg);

            if (has_goal) {
                execute_astar_planning();
            }
        }

        void execute_astar_planning()
        {
            if (!has_odom) return;

            GridPoint start = worldToGrid(input_msg.pose.pose.position.x, input_msg.pose.pose.position.y);
            GridPoint goal = worldToGrid(global_goal_x, global_goal_y);

            start.x = std::clamp(start.x, 0, map_width - 1);
            start.y = std::clamp(start.y, 0, map_height - 1);
            goal.x = std::clamp(goal.x, 0, map_width - 1);
            goal.y = std::clamp(goal.y, 0, map_height - 1);

            if (start == goal) return;

            auto comp = [](const AStarNode& a, const AStarNode& b) { return a.f > b.f; };
            std::priority_queue<AStarNode, std::vector<AStarNode>, decltype(comp)> open_list(comp);
            std::map<GridPoint, double> closed_g;
            std::map<GridPoint, GridPoint> parent_track;

            AStarNode start_node = {start, 0.0, std::hypot(start.x - goal.x, start.y - goal.y), 0.0, start};
            start_node.f = start_node.g + start_node.h;
            
            open_list.push(start_node);
            bool found = false;
            size_t iterations = 0;

            int dx[] = {-1, 1, 0, 0, -1, -1, 1, 1};
            int dy[] = {0, 0, -1, 1, -1, 1, -1, 1};

            while (!open_list.empty() && rclcpp::ok()) {
                AStarNode current = open_list.top();
                open_list.pop();

                if (current.pt == goal) { found = true; break; }
                if (++iterations > 800) break; 

                closed_g[current.pt] = current.g;

                for (int i = 0; i < 8; ++i) {
                    GridPoint next_pt = {current.pt.x + dx[i], current.pt.y + dy[i]};
                    
                    if (next_pt.x < 0 || next_pt.x >= map_width || next_pt.y < 0 || next_pt.y >= map_height) continue;
                    
                    size_t idx = static_cast<size_t>(next_pt.y * map_width + next_pt.x);
                    if (map_msg.data[idx] >= 50) continue;

                    double step_cost = (dx[i] != 0 && dy[i] != 0) ? 1.414 : 1.0;
                    double next_g = current.g + step_cost;

                    if (closed_g.count(next_pt) && next_g >= closed_g[next_pt]) continue;

                    double h = std::hypot(next_pt.x - goal.x, next_pt.y - goal.y);
                    AStarNode neighbor = {next_pt, next_g, h, next_g + h, current.pt};
                    
                    parent_track[next_pt] = current.pt;
                    closed_g[next_pt] = next_g;
                    open_list.push(neighbor);
                }
            }

            if (found) {
                calculated_path.poses.clear();
                calculated_path.header.stamp = this->get_clock()->now();
                calculated_path.header.frame_id = "odom";

                GridPoint curr_trace = goal;
                while (!(curr_trace == start)) {
                    geometry_msgs::msg::PoseStamped p;
                    p.header.frame_id = "odom";
                    gridToWorld(curr_trace.x, curr_trace.y, p.pose.position.x, p.pose.position.y);
                    calculated_path.poses.push_back(p);
                    curr_trace = parent_track[curr_trace];
                }
                std::reverse(calculated_path.poses.begin(), calculated_path.poses.end());
                path_publisher->publish(calculated_path);
                
                current_waypoint_idx = std::min(calculated_path.poses.size() - 1, static_cast<size_t>(2)); 
                last_path_update_time_ = this->get_clock()->now(); 
                has_path = true;
            } else {
                has_path = false;
            }
        }

        void callback_control_loop()
        {
            // SAFETY WATCHDOG: Kill motors if A* thread has frozen for over 1.0 seconds
            if (has_path) {
                auto time_since_update = (this->get_clock()->now() - last_path_update_time_).seconds();
                if (time_since_update > 1.0) {
                    publish_velocity(0.0, 0.0);
                    return;
                }
            }

            if (!has_path || calculated_path.poses.empty()) {
                publish_velocity(0.0, 0.0);
                return;
            }

            double curr_x = input_msg.pose.pose.position.x;
            double curr_y = input_msg.pose.pose.position.y;
            
            double q_x = input_msg.pose.pose.orientation.x;
            double q_y = input_msg.pose.pose.orientation.y;
            double q_z = input_msg.pose.pose.orientation.z;
            double q_w = input_msg.pose.pose.orientation.w;
            double curr_yaw = std::atan2(2.0 * (q_w * q_z + q_x * q_y), 1.0 - 2.0 * (q_y * q_y + q_z * q_z));

            double final_dist_to_goal = std::hypot(global_goal_x - curr_x, global_goal_y - curr_y);

            if (final_dist_to_goal < 0.15) {
                RCLCPP_INFO(this->get_logger(), "Waypoint reached at [%.2f, %.2f]!", global_goal_x, global_goal_y);
                
                // 1. Check if we have more waypoints waiting in the line
                if (!waypoint_queue_.empty()) {
                    loadNextWaypoint(); 
                    // Force A* to calculate a brand new route from here to the new waypoint
                    has_path = false; 
                } else {
                    // 2. The queue is completely empty. The mission is truly over.
                    publish_velocity(0.0, 0.0);
                    has_goal = false; 
                    has_path = false;
                    mission_complete_ = true;
                    
                    RCLCPP_INFO(this->get_logger(), "Final goal reached successfully! Mission Complete. Stopping motors.");
                }
            }
                        else {
                double look_ahead_x = calculated_path.poses[current_waypoint_idx].pose.position.x;
                double look_ahead_y = calculated_path.poses[current_waypoint_idx].pose.position.y;
                double waypoint_dist = std::hypot(look_ahead_x - curr_x, look_ahead_y - curr_y);

                double angle_to_goal = std::atan2(look_ahead_y - curr_y, look_ahead_x - curr_x);
                double angle_error = std::atan2(std::sin(angle_to_goal - curr_yaw), std::cos(angle_to_goal - curr_yaw));

                double raw_angular_velocity = Kp_ang * angle_error;
                double out_angular = std::clamp(raw_angular_velocity, -1.0, 1.0);
                double out_linear = 0.0;

                if (std::abs(angle_error) <= 0.78) {
                    out_linear = std::min(0.14, Kp_lin * waypoint_dist); 
                }
                
                publish_velocity(out_linear, out_angular);
            }
        }
    };
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    std::cout << "--- STANDALONE REALTIME A* NAVIGATION ENGINE ONLINE ---" << std::endl;
    rclcpp::spin(std::make_shared<turtle_nav::MyNode>(options));
    rclcpp::shutdown();
    return 0;
}
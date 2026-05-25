#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import TwistStamped, PoseArray
from nav_msgs.msg import Odometry
from modules import dubins_path_planner as dpp
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np

PI = 3.14159

class DubinsPathNode(Node):
    def __init__(self):
        super().__init__("dubins_path")

        # Initialize subscriber
        self.pose_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Initialize publisher
        self.velocity_publisher = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )

        self.obstacle_subscriber_ = self.create_subscription(
            PoseArray,
            '/obstacles/poses',
            self.obstacle_callback,
            10
        )

        self.main_loop_timer = self.create_timer(0.01, self.main_loop)

        # Keep track of the latest input message (initialized to None or empty object)
        self.odom_msg = None

        # Initialize timer (100ms = 0.1 seconds)
        self.publish_timer = self.create_timer(0.01, self.callback_time)

        # Target Goal & PID Gains
        self.goal_x = 4.0
        self.goal_y = 2.0
        self.kp_lin = 1.0
        self.kp_ang = 1.0
        self.wp_x = self.goal_x
        self.wp_y = self.goal_y
        self.is_wp_reached = False
        self.obstacles = None

    def odom_callback(self, msg):
        self.odom_msg = msg
        self.curr_x = self.odom_msg.pose.pose.position.x
        self.curr_y = self.odom_msg.pose.pose.position.y

        # Extract quaternions
        q_x = self.odom_msg.pose.pose.orientation.x
        q_y = self.odom_msg.pose.pose.orientation.y
        q_z = self.odom_msg.pose.pose.orientation.z
        q_w = self.odom_msg.pose.pose.orientation.w
        
        # Robust Quaternion-to-Yaw conversion (handling full 3D rotations gracefully)
        siny_cosp = 2.0 * (q_w * q_z + q_x * q_y)
        cosy_cosp = 1.0 - 2.0 * (q_y * q_y + q_z * q_z)
        self.curr_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def obstacle_callback(self, msg):
        self.obstacles:PoseArray = msg

    def main_loop(self):
        o_P_new = self.obstacle_handler()
        if(o_P_new is not None ):
            self.wp_x, self.wp_y = o_P_new
            self.get_logger().info("Performing obstacle avoidance manoeuvers")
        if(self.is_wp_reached):
            self.wp_x = self.goal_x
            self.wp_y = self.goal_y
            self.is_wp_reached = False
            self.get_logger().info("going back to the goal")



    def obstacle_handler(self):
        if self.odom_msg is None:
            self.get_logger().info('Waiting for initial odometry...', once=True)
            return
        
        if self.obstacles is None:
            return
        
        for obstacle in self.obstacles.poses:
            obs_x = obstacle.position.x
            obs_y = obstacle.position.y
            range = math.sqrt((obs_x)**2 + (obs_y)**2)
            # self.get_logger().info(f"range: {range}")
            alpha = math.atan2(obs_y, obs_x)
            if (range < 0.5 and alpha >= -PI/4 and alpha <= PI/4):
                # self.get_logger().info(f"Current range: {range}")
                d_avoid = 0.4
                turn_angle = -PI/1.8 if obs_y > 0 else PI/1.8
                m_x_new = d_avoid*math.cos(turn_angle + alpha)
                m_y_new = d_avoid*math.sin(turn_angle + alpha)
                o_R_m = np.array([[math.cos(self.curr_yaw), -math.sin(self.curr_yaw)],
                                  [math.sin(self.curr_yaw), math.cos(self.curr_yaw)]])
                o_P_new = np.array([[self.curr_x, self.curr_y]]).T + o_R_m@np.array([[m_x_new, m_y_new]]).T
                o_x_new = o_P_new[0,0]
                o_y_new = o_P_new[1,0]
                return (o_x_new, o_y_new)

       
    def callback_time(self):
        # Prevent calculations if we haven't received an /odom message yet
        if self.odom_msg is None:
            self.get_logger().info('Waiting for initial odometry...', once=True)
            return

        vel_msg = TwistStamped()
        
        # Populate headers (Crucial for Stamped messages)
        vel_msg.header.stamp = self.get_clock().now().to_msg()
        vel_msg.header.frame_id = 'base_link'

        # Current positional coordinates
        curr_x = self.odom_msg.pose.pose.position.x
        curr_y = self.odom_msg.pose.pose.position.y

        # Extract quaternions
        q_x = self.odom_msg.pose.pose.orientation.x
        q_y = self.odom_msg.pose.pose.orientation.y
        q_z = self.odom_msg.pose.pose.orientation.z
        q_w = self.odom_msg.pose.pose.orientation.w
        
        # Robust Quaternion-to-Yaw conversion (handling full 3D rotations gracefully)
        siny_cosp = 2.0 * (q_w * q_z + q_x * q_y)
        cosy_cosp = 1.0 - 2.0 * (q_y * q_y + q_z * q_z)
        curr_yaw = math.atan2(siny_cosp, cosy_cosp)

        # Errors
        dist_err = self.calculate_distance(curr_x, curr_y)
        angle_to_goal = math.atan2(self.wp_y - curr_y, self.wp_x - curr_x)
        
        angle_error = angle_to_goal - curr_yaw
        
        # Normalize to [-pi, pi]
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        # Control Logic
        if dist_err < 0.3:
            vel_msg.twist.linear.x = 0.0
            vel_msg.twist.angular.z = 0.0
            # Throttled logging to avoid console spam (prints once every 1 sec)
            self.get_logger().info('Goal Reached!', throttle_duration_sec=1.0)
            self.is_wp_reached = True
        else:
            vel_msg.twist.linear.x = min(0.4, self.kp_lin * dist_err)
            vel_msg.twist.angular.z = self.kp_ang * angle_error
        
        self.velocity_publisher.publish(vel_msg)

    def calculate_distance(self, curr_x, curr_y):
        return math.sqrt((self.wp_x - curr_x)**2 + (self.wp_y - curr_y)**2)

def main(args=None):
    rclpy.init(args=args)
    node = DubinsPathNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
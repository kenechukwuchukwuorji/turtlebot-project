#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import TwistStamped, PoseArray, Pose
from nav_msgs.msg import Odometry
import numpy as np
from scipy.spatial.distance import cdist

class MapSearchNode(Node):
    def __init__(self):
        super().__init__("map_search")

        self.pose_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.obstacle_subscriber_ = self.create_subscription(
            PoseArray,
            '/obstacles/poses',
            self.obstacle_callback,
            10
        )

        self.map_publisher = self.create_publisher(PoseArray, "map_grid", 10)

        self.main_loop_timer = self.create_timer(0.01, self.main_loop)
        self.map = self.create_map()
        self.odom_msg = None
        self.obstacles = None

    def obstacle_callback(self, msg):
        self.obstacles:PoseArray = msg

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

    def main_loop(self):
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
            
            o_R_m = np.array([[math.cos(self.curr_yaw), -math.sin(self.curr_yaw)],
                                [math.sin(self.curr_yaw), math.cos(self.curr_yaw)]])
            o_P_obs = np.array([[self.curr_x, self.curr_y]]).T + o_R_m@np.array([[obs_x, obs_y]]).T
            o_x_obs = o_P_obs[0,0]
            o_y_obs = o_P_obs[1,0]
            self.update_map(o_x_obs, o_y_obs)

        map_msg = PoseArray()
        map_list = []
        for i in self.map:
            map_pose = Pose()
            map_pose.position.x = i[0]
            map_pose.position.y = i[1]
            map_list.append(map_pose)

        map_msg.poses = map_list
        self.map_publisher.publish(map_msg)


    def update_map(self, x, y):
        point = np.array([[x, y]])
        distances = cdist(point, self.map)
        threshold = 3
        self.map = self.map[distances[0] > threshold]

    def create_map(self):
        # Map configuration
        map_size = 10.0        # Length of one side of the square map
        num_points_per_side = 11  # Number of points along each axis (e.g., 0 to 10 inclusive)

        # 1. Create evenly spaced intervals for both axes
        x_range = np.linspace(0.0, map_size, num_points_per_side)
        y_range = np.linspace(0.0, map_size, num_points_per_side)

        # 2. Generate a coordinate matrices grid
        X, Y = np.meshgrid(x_range, y_range)

        # 3. Stack them into a clean iterable array of (x, y) pairs
        # .T ensures it increments cleanly from the bottom-left corner outward
        grid_points_array = np.vstack([X.ravel(), Y.ravel()]).T

        return grid_points_array

def main(args=None):
    rclpy.init(args=args)
    node = MapSearchNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
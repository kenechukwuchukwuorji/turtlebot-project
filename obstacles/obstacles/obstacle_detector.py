#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose, Twist
from visualization_msgs.msg import Marker, MarkerArray
import math

#this is my node template

class ObstacleDetector(Node):


    def __init__(self):
        super().__init__("obstacle_detector")
        self.obstacle_distance_threshold = 1.5 #Distance for points to be considered as obstacles
        self.cluster_distance_threshold = 0.2 #max. distance between points to be considered as part of the same cluster(obstacle)
        self.min_cluster_size = 3 #minimum number of points for a cluster to be considered as an obstacle
        self.avoidance_range    = 0.5
        self.linear_speed       = 0.15
        self.angular_speed      = 0.5

        self.scan_sub = self.create_subscription(LaserScan, "/scan",
                                                     self.scan_callback, 10)
        self.obstacle_pub = self.create_publisher(PoseArray, '/obstacles/poses', 10)

        self.pub_markers = self.create_publisher(MarkerArray, "/obstacle_markers", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)


    def scan_callback(self, msg:LaserScan):

        points = []
        for i, range in enumerate(msg.ranges):
            if msg.range_min < range < self.obstacle_distance_threshold:
                angle = msg.angle_min + i * msg.angle_increment
                x = range * math.cos(angle)
                y = range * math.sin(angle)
                points.append((x, y))

        obstacle = self.cluster_points(points)

        obstacle_positions = []
        for cluster in obstacle:
                    centroid_x = sum(p[0] for p in cluster) / len(cluster)
                    centroid_y = sum(p[1] for p in cluster) / len(cluster)
                    obstacle_positions.append((centroid_x, centroid_y))
                    self.get_logger().info(f'Obstacle at x={centroid_x:.2f}m, y={centroid_y:.2f}m ')                      )

        self.publish_obstacle_positions(obstacle_positions)
        self.avoid_obstacles(obstacle_positions)

    def cluster_points(self, points):

        if not points:
            return []

        clusters = []
        current_cluster = [points[0]]

        for i in range(1, len(points)):
            dist_x = points[i][0] - points[i-1][0]
            dist_y = points[i][1] - points[i-1][1]
            dist = math.hypot(dist_x, dist_y)

            if dist < self.cluster_distance_threshold:
                current_cluster.append(points[i])
            else:
                if len(current_cluster) >= self.min_cluster_size:
                    clusters.append(current_cluster)
                current_cluster = [points[i]]

        if len(current_cluster) >= self.min_cluster_size:
            clusters.append(current_cluster)

        return clusters

    def publish_obstacle_positions(self, obstacle_positions):
        pose_array = PoseArray()

        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = 'base_link'

        for (centroid_x, centroid_y) in obstacle_positions:
            pose = Pose()
            pose.position.x = centroid_x
            pose.position.y = centroid_y
            pose.position.z = 0.0
            pose_array.poses.append(pose)

        self.obstacle_pub.publish(pose_array)
        self.get_logger().info(f'Published {len(obstacle_positions)} obstacles')

    # def publish_markers(self, obstacle_positions, header):
    #     marker_array = MarkerArray()
    #     for i, (centroid_x, centroid_y) in enumerate(obstacle_positions):
    #         marker = Marker()
    #         marker.header = header
    #         marker.header.frame_id = "base_link"
    #         marker.ns = "obstacles"
    #         marker.id = i
    #         marker.type = Marker.SPHERE
    #         marker.action = Marker.ADD
    #         marker.pose.position.x = centroid_x
    #         marker.pose.position.y = centroid_y
    #         marker.pose.position.z = 0.1
    #         marker.scale.x = 0.2
    #         marker.scale.y = 0.2
    #         marker.scale.z = 0.2
    #         marker.color.r = 1.0
    #         marker.color.a = 0.8
    #         marker_array.markers.append(marker)
    #     for j in range(len(obstacle_positions), 20):
    #         clear = Marker()
    #         clear.header = header
    #         clear.ns = 'obstacles'
    #         clear.id = j
    #         clear.action = Marker.DELETE
    #         marker_array.markers.append(clear)
    #     self.pub_markers.publish(marker_array)

    def avoid_obstacles(self, obstacle_positions):
        cmd = Twist()

        if not obstacle_positions:
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            return

        closest = min(obstacle_positions, key=lambda p: math.hypot(p[0], p[1]))
        centroid_x, centroid_y = closest
        distance = math.hypot(centroid_x, centroid_y)

        if distance < self.avoidance_range:
            cmd.linear.x = 0.0
            if centroid_y >= 0:
                cmd.angular.z = -self.angular_speed  
            else:
                cmd.angular.z = self.angular_speed  
            self.get_logger().info(
                f'AVOIDING obstacle at ({centroid_x:.2f}, {centroid_y:.2f}) '
                f'— {distance:.2f}m away, turning {"right" if centroid_y >= 0 else "left"}'
            )
        else:
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node) #keeps te node runninr and allows to process callback
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

#this is my node template

class ObstacleDetector(Node):


    def __init__(self):

        self.obstacle_distance_threshold = 1.5 #Distance for points to be considered as obstacles
        self.cluster_distance_threshold = 0.2 #max. distance between points to be considered as part of the same cluster(obstacle)
        self.min_cluster_size = 3 #minimum number of points for a cluster to be considered as an obstacle
        super().__init__("obstacle_detector")
        self.scan_sub = self.create_subscription(LaserScan, "/scan", 
                                                     self.scan_callback, 10)
        
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
                    self.get_logger().info(
                        f'Obstacle at x={centroid_x:.2f}m, y={centroid_y:.2f}m '
                        f'({math.degrees(math.atan2(centroid_y, centroid_x)):.1f}°, '
                        f'{math.hypot(centroid_x, centroid_y):.2f}m away)'
                    )

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

 


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node) #keeps te node runninr and allows to process callback
    rclpy.shutdown()


if __name__ == "__main__":
    main()
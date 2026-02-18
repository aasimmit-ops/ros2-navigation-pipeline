#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class TrajectoryNode(Node):

    def __init__(self):
        super().__init__('trajectory_node')

        self.subscription = self.create_subscription(
            Path,
            '/smooth_path',
            self.path_callback,
            10)

        self.publisher = self.create_publisher(
            Path,
            '/trajectory',
            10)

        self.velocity = 0.5          # m/s
        self.sample_distance = 0.05  # meters

        self.get_logger().info("Trajectory Node Started")

    def path_callback(self, msg):

        if len(msg.poses) < 2:
            self.get_logger().warn("Not enough points in smooth path.")
            return

        # Extract points
        points = []
        for pose in msg.poses:
            points.append([
                pose.pose.position.x,
                pose.pose.position.y
            ])

        points = np.array(points)

        # Resample path uniformly
        resampled = [points[0]]
        accumulated = 0.0

        for i in range(1, len(points)):
            dx = points[i][0] - points[i-1][0]
            dy = points[i][1] - points[i-1][1]
            distance = np.sqrt(dx**2 + dy**2)

            accumulated += distance

            if accumulated >= self.sample_distance:
                resampled.append(points[i])
                accumulated = 0.0

        # Create time-parameterized trajectory
        trajectory_msg = Path()
        trajectory_msg.header.frame_id = "map"
        trajectory_msg.header.stamp = self.get_clock().now().to_msg()

        cumulative_time = 0.0
        previous_point = resampled[0]

        for point in resampled:
            dx = point[0] - previous_point[0]
            dy = point[1] - previous_point[1]
            distance = np.sqrt(dx**2 + dy**2)

            if distance > 0:
                dt = distance / self.velocity
                cumulative_time += dt

            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()

            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            pose.pose.orientation.w = 1.0

            trajectory_msg.poses.append(pose)

            previous_point = point

        self.publisher.publish(trajectory_msg)

        self.get_logger().info(
            f"Published trajectory with {len(trajectory_msg.poses)} points"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

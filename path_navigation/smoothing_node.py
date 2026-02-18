#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from scipy.interpolate import CubicSpline

from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Path


class PathSmoothingNode(Node):

    def __init__(self):
        super().__init__('path_smoothing_node')

        self.subscription = self.create_subscription(
            PoseArray,
            '/waypoints',
            self.waypoint_callback,
            10)

        self.publisher = self.create_publisher(
            Path,
            '/smooth_path',
            10)

        self.get_logger().info("Path Smoothing Node Started")

    def waypoint_callback(self, msg):

        if len(msg.poses) < 2:
            self.get_logger().warn("Not enough waypoints to smooth.")
            return

        # Extract x and y
        waypoints = []
        for pose in msg.poses:
            waypoints.append([pose.position.x, pose.position.y])

        waypoints = np.array(waypoints)

        # Compute cumulative arc length
        s = np.zeros(len(waypoints))
        for i in range(1, len(waypoints)):
            s[i] = s[i-1] + np.linalg.norm(waypoints[i] - waypoints[i-1])

        # Create splines
        cs_x = CubicSpline(s, waypoints[:, 0])
        cs_y = CubicSpline(s, waypoints[:, 1])

        # Sample many points
        s_fine = np.linspace(0, s[-1], 500)
        x_smooth = cs_x(s_fine)
        y_smooth = cs_y(s_fine)

        # Create Path message
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for x, y in zip(x_smooth, y_smooth):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.publisher.publish(path_msg)
        self.get_logger().info("Published smoothed path")


def main(args=None):
    rclpy.init(args=args)
    node = PathSmoothingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

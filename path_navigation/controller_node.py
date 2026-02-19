#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import math

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class ControllerNode(Node):

    def __init__(self):
        super().__init__('controller_node')

        self.trajectory = []
        self.current_pose = None

        # -----------------------------
        # Pure Pursuit Parameters
        # -----------------------------
        self.lookahead_distance = 0.7
        self.linear_velocity = 0.18
        self.max_angular_velocity = 1.0

        # -----------------------------
        # Obstacle Avoidance Parameters
        # -----------------------------
        self.obstacle_detected = False
        self.min_obstacle_distance = 0.5   # meters
        self.avoidance_turn_speed = 0.6

        # -----------------------------
        # Subscribers
        # -----------------------------
        self.create_subscription(
            Path,
            '/trajectory',
            self.trajectory_callback,
            10)

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)

        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        # -----------------------------
        # Publisher
        # -----------------------------
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Controller Node with Obstacle Avoidance Started")

    # -----------------------------
    # Callbacks
    # -----------------------------
    def trajectory_callback(self, msg):
        if msg.poses is None or len(msg.poses) == 0:
            self.get_logger().warn("Received empty trajectory.")
            self.trajectory = []
        else:
            self.trajectory = msg.poses

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def scan_callback(self, msg):

        if msg.ranges is None or len(msg.ranges) == 0:
            self.obstacle_detected = False
            return

        total_ranges = len(msg.ranges)
        center = total_ranges // 2
        window = total_ranges // 6  # ~60 degree sector

        front_ranges = msg.ranges[center - window: center + window]

        valid_ranges = [
            r for r in front_ranges
            if not np.isinf(r) and not np.isnan(r)
        ]

        if len(valid_ranges) == 0:
            self.obstacle_detected = False
            return

        if min(valid_ranges) < self.min_obstacle_distance:
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

    # -----------------------------
    # Main Control Loop
    # -----------------------------
    def control_loop(self):

        # -----------------------------
        # SAFETY CHECKS
        # -----------------------------
        if self.lookahead_distance <= 0:
            self.get_logger().error("Lookahead distance must be positive.")
            return

        if self.current_pose is None:
            self.get_logger().warn("Waiting for odometry...")
            return

        if len(self.trajectory) == 0:
            self.get_logger().warn("No trajectory available. Stopping robot.")
            self.publish_stop()
            return

        # -----------------------------
        # OBSTACLE OVERRIDE (Highest Priority)
        # -----------------------------
        if self.obstacle_detected:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = self.avoidance_turn_speed
            self.cmd_pub.publish(cmd)
            return

        # -----------------------------
        # PURE PURSUIT
        # -----------------------------
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y

        orientation_q = self.current_pose.orientation

        siny_cosp = 2 * (orientation_q.w * orientation_q.z)
        cosy_cosp = 1 - 2 * (orientation_q.z * orientation_q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)

        target = None

        for pose in self.trajectory:
            dx = pose.pose.position.x - robot_x
            dy = pose.pose.position.y - robot_y
            distance = math.sqrt(dx**2 + dy**2)

            if distance >= self.lookahead_distance:
                target = pose
                break

        # If no target → stop safely
        if target is None:
            self.get_logger().info("Reached end of trajectory.")
            self.publish_stop()
            return

        dx = target.pose.position.x - robot_x
        dy = target.pose.position.y - robot_y

        # Transform to robot frame
        x_r = math.cos(theta) * dx + math.sin(theta) * dy
        y_r = -math.sin(theta) * dx + math.cos(theta) * dy

        # Safe curvature calculation
        if self.lookahead_distance == 0:
            self.publish_stop()
            return

        curvature = (2 * y_r) / (self.lookahead_distance ** 2)
        angular_velocity = self.linear_velocity * curvature

        # Limit angular velocity
        angular_velocity = max(
            min(angular_velocity, self.max_angular_velocity),
            -self.max_angular_velocity
        )

        cmd = Twist()
        cmd.linear.x = self.linear_velocity
        cmd.angular.z = angular_velocity

        self.cmd_pub.publish(cmd)

    # -----------------------------
    # Helper Function
    # -----------------------------
    def publish_stop(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

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
        self.trajectory = msg.poses

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def scan_callback(self, msg):
        # Check front 60 degrees of scan
        total_ranges = len(msg.ranges)
        center = total_ranges // 2
        window = total_ranges // 6  # ~60 degree sector

        front_ranges = msg.ranges[center - window : center + window]

        valid_ranges = [r for r in front_ranges if not np.isinf(r)]

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

        if self.current_pose is None or len(self.trajectory) == 0:
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

        # If no target → stop
        if target is None:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            return

        dx = target.pose.position.x - robot_x
        dy = target.pose.position.y - robot_y

        # Transform to robot frame
        x_r = math.cos(theta) * dx + math.sin(theta) * dy
        y_r = -math.sin(theta) * dx + math.cos(theta) * dy

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


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

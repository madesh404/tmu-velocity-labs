#!/usr/bin/env python3  
import rclpy
from rclpy.node import Node

import numpy as np

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """

    def __init__(self):
        super().__init__('safety_node')

        # Store the car's current forward speed.
        self.speed = 0.0

        # Listen for the car's current velocity.
        self.odom_sub = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10
        )

        # Listen for LiDAR measurements.
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Allow this node to send driving commands to the car.
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )

    def odom_callback(self, odom_msg):
        # Save the car's current forward velocity.
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # Convert the LiDAR ranges into a NumPy array.
        ranges = np.array(scan_msg.ranges)

        # Calculate the angle corresponding to every LiDAR beam.
        angles = (
            scan_msg.angle_min
            + np.arange(len(ranges)) * scan_msg.angle_increment
        )

        # Calculate how quickly the car is approaching an obstacle
        # along each LiDAR beam.
        closing_speed = self.speed * np.cos(angles)

        # Only use measurements that:
        # 1. are finite (not inf or nan)
        # 2. have a positive distance
        # 3. correspond to an obstacle we are approaching
        valid = (
            np.isfinite(ranges)
            & (ranges > 0.0)
            & (closing_speed > 0.0)
        )

        # Start by assuming every beam has infinite TTC.
        ttc = np.full(len(ranges), np.inf)

        # For valid beams:
        # TTC = distance / closing speed
        ttc[valid] = ranges[valid] / closing_speed[valid]

        # If the smallest TTC is below our threshold,
        # tell the car to stop.
        if np.min(ttc) < 1.0:
            brake_msg = AckermannDriveStamped()
            brake_msg.drive.speed = 0.0

            self.drive_pub.publish(brake_msg)


def main(args=None):
    rclpy.init(args=args)

    safety_node = SafetyNode()

    rclpy.spin(safety_node)

    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
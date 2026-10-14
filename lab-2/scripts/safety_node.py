
"""Automatic Emergency Braking using instantaneous Time to Collision (iTTC). """

import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


class SafetyNode(Node):
	"""Brakes the car when the iTTC of any scan beam falls below a threshold."""

	def __init__(self):
		super().__init__('safety_node')
		
		# Tunable parameters (overrides with --ros-args -p ttc_threshold:=0.6)
		self.declare_parameter('ttc_threshold', 0.5) # seconds
		self.declare_parameter('min_speed', 0.2) # m/s, ignore braking below this
		self.ttc_threshold = self.get_parameter('ttc_threshold').value
		self.min_speed = self.get_parameter('min_speed').value

		# Longitudinal velocity, updated from odometry
		self.speed = 0.0

		self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
		self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
		self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

	def odom_callback(self, odom_msg: Odometry):
		self.speed = odom_msg.twist.twist.linear.x

	def scan_callback(self, scan_msg: LaserScan):
		ranges = np.asarray(scan_msg.ranges, dtype=np.float64)
		angles = scan_msg.angle_min + np.arange(ranges.size) * scan_msg.angle_increment
		
		# Drop nan/inf readings and anything outside the sensor's valid range
		valid = (np.isfinite(ranges)
			& (ranges >= scan_msg.range_min)
			& (ranges <= scan_msg.range_max))

		# Range rate for each beam: r_dot = -v_x * cos(theta)
		# (negative when the range is shrinking)
		# {-r_dot+ = max(v_x * cos(theta), 0), so beams where the range is
		# growing fet a zero denominator and an iTTC of infinity.

		closing_speed = np.maximum(self.speed * np.cos(angles), 0.0)
		closing = closing_speed > 1e-3

		mask = valid & closing
		if not np.any(mask):
			return # nothing is approaching, so no collision is imminent

		ttc = ranges[mask] / closing_speed[mask]
		min_ttc = float(np.min(ttc))

		# Ignore very low speeds to avoid false positives when creeping or stopped
		if abs(self.speed) > self.min_speed and min_ttc < self.ttc_threshold:
			self.brake()
			self.get_logger().info(f'Emergency brake! min iTTC = {min_ttc:.2f}s at speed {self.speed:.2f} m/s')
	
	def brake(self):
		msg = AckermannDriveStamped()
		msg.header.stamp = self.get_clock().now().to_msg()
		msg.driver.speed = 0.0
		self.drive_pub.publish(msg)

def main(args=None):
	rclpy.init(args=args)
	node = SafetyNode()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__=='__main__':
	main()

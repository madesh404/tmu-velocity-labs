import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: create subscribers and publishers
        self.scan_sub = self.create_subscription(LaserScan, lidarscan_topic, self.scan_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        # TODO: set PID gains
        self.kp = 0.3 
        self.kd = 0.0
        self.ki = 0.0

        # TODO: store history
        self.integral = 0.0
        self.prev_error = 0.0 
        self.error = 0.0
        self.prev_time = None

        # TODO: store any necessary values you think you'll need
        self.desired_distance = 1.0
        self.lookahead = 1.5
        self.theta = np.radians(45)
        self.max_steering = np.radians(30)
    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """

        #TODO: implement
        if self.angle_increment == 0.0:
            return None

        index = int(round((angle - self.angle_min) / self.angle_increment))

        if index < 0 or index >= len(range_data):
            return None

        distance = range_data[index]

        if not np.isfinite(distance):
            return None

        if distance < self.range_min or distance > self.range_max:
            return None

        return float(distance)

    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """

        #TODO:implement
        b = self.get_range(range_data, np.radians(90))
        a = self.get_range(range_data, np.radians(90) - self.theta)
        print("LiDAR a:", a, "LiDAR b:", b)
        if a is None or b is None:
            return None

        alpha = np.arctan2(b - a * np.cos(self.theta), a * np.sin(self.theta))
        current_distance = b * np.cos(alpha)

        future_distance = current_distance - self.lookahead * np.sin(alpha)
        error = dist - future_distance
        return float(error)


    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        angle = 0.0
        # TODO: Use kp, ki & kd to implement a PID controller
        current_time = self.get_clock().now().nanoseconds / 1e9

        if self.prev_time is None:
            dt = 0.0
        else:
            dt = current_time - self.prev_time

        proportional = self.kp * error
        derivative = 0.0

        if dt > 0.0:
            self.integral += error * dt
            derivative = (error - self.prev_error) / dt

        integral_term = self.ki * self.integral
        derivative_term = self.kd * derivative

        angle = -(proportional + integral_term + derivative_term)
        angle = float(np.clip(angle, -self.max_steering, self.max_steering))
        print("Error:", round(error, 3), "Steering:", round(angle, 3), flush=True)
        self.prev_error = error
        self.error = error
        self.prev_time = current_time

        drive_msg = AckermannDriveStamped()
        # TODO: fill in drive message and publish
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = float(velocity)
        self.drive_pub.publish(drive_msg)
    def scan_callback(self, msg):
        error = 0.0 # TODO: replace with error calculated by get_error()
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment
        self.range_min = msg.range_min
        self.range_max = msg.range_max
        error = self.get_error(msg.ranges, self.desired_distance)

        if error is None:
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.steering_angle = 0.0
            drive_msg.drive.speed = 0.0
            self.drive_pub.publish(drive_msg)
            self.prev_time = None
            return

        velocity = 0.0 # TODO: calculate desired car velocity based on error
        estimated_angle = abs(self.kp * error)

        if estimated_angle < np.radians(10):
            velocity = 1.5
        elif estimated_angle < np.radians(20):
            velocity = 1.0
        else:
            velocity = 0.5

        self.pid_control(error, velocity) # TODO: actuate the car with PID


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
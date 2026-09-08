import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.logging import set_logger_level, LoggingSeverity
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

np.set_printoptions(
    2, suppress=True
)  # Print numpy arrays to specified d.p. and suppress scientific notation (e.g. 1e-5)

max_translate_velocity = 0.4 # Can be implemented as parameter
max_turn_velocity = max_translate_velocity * 2 # Can be implemented as parameter
set_logger_level("obstacle_avoidance", level=LoggingSeverity.DEBUG) # Configure to either LoggingSeverity.INFO or LoggingSeverity.DEBUG  

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        """Node constructor"""
        super().__init__("obstacle_avoidance")
        self.get_logger().info("Starting Obstacle Avoidance")

        self.pub_cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)  # Publish to cmd_vel node
        self.sub_scan = self.create_subscription(LaserScan, "scan", self.sub_scan_callback, 2) # The subscriber to the Lidar ranges.
        self.last_scan = None # Copied laser scan message

        self.timer = self.create_timer(0.05, self.timer_callback)  # Runs at 20Hz. Can be changed.

    def move_2D(self, x: float = 0.0, y: float = 0.0, turn: float = 0.0):
        """Publishes a twist command to move in 2D space. +ve x is forwards, +ve y is left, and +ve turn is anticlockwise"""
        twist_msg = Twist()
        x = np.clip(x, -max_translate_velocity, max_translate_velocity)
        y = np.clip(y, -max_translate_velocity, max_translate_velocity)
        turn = np.clip(turn, -max_translate_velocity*2, max_translate_velocity*2)
        twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z = float(x), float(y), 0.0
        twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z = 0.0, 0.0, float(turn)
        self.pub_cmd_vel.publish(twist_msg)

    def sub_scan_callback(self, msg):
        """Scan subscriber"""
        self.last_scan = np.array(msg.ranges)[::20] # Slices the 721 scan array to return only 36 scans. Feel free to edit

    def timer_callback(self):
        """Controller loop"""

        if self.last_scan is None:
            return # Does not run if the laser message is not received.
        
        ranges = np.array(self.last_scan)
        # Filter out invalid readings (0.0, nan, or inf)
        ranges = np.where(np.isnan(ranges) | np.isinf(ranges) | (ranges <= 0.05), 10.0, ranges)

        # In 36-scan slice (each index ~ 10 degrees):
        # Front-left: indices [0:4] (0° to ~30° CCW)
        # Front-right: indices [32:36] (~-40° to 0°)
        front_left = np.min(ranges[0:4])
        front_right = np.min(ranges[32:])
        front_dist = min(front_left, front_right)

        safe_distance = 0.6  # Safe clearance distance in meters

        if front_dist < safe_distance:
            # Obstacle detected in front -> steer towards the clearer side
            if front_left < front_right:
                # Closer on the left: turn right (-turn) or strafe right (-y)
                self.move_2D(x=0.05, y=-0.15, turn=-0.6)
            else:
                # Closer on the right: turn left (+turn) or strafe left (+y)
                self.move_2D(x=0.05, y=0.15, turn=0.6)
        else:
            # Path ahead is clear -> drive forward towards the goal
            self.move_2D(x=0.3, y=0.0, turn=0.0)


def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance_node = ObstacleAvoidanceNode()
    rclpy.spin(obstacle_avoidance_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
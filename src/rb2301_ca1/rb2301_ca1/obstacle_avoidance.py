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

        #parameters
        self.N = 36  # Number of laser scans to use
        self.angles = None  # Angles corresponding to the downsampled laser scans
        self.scan_yaw_offset = np.pi  # The laser is mounted facing backwards
        self.danger_threshold = 0.2
        self.avoidance_hold_cycles = 20  # Keep avoiding for 1 second at 20 Hz
        self.avoidance_cycles = 0
        self.avoidance_angle = 0.0

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
        ranges = np.asarray(msg.ranges, dtype=float)
        indices = np.linspace(0, len(ranges) - 1, self.N, dtype=int)
        self.last_scan = ranges[indices]
        raw_angles = msg.angle_min + indices * msg.angle_increment + self.scan_yaw_offset
        self.angles = np.arctan2(np.sin(raw_angles), np.cos(raw_angles))

    def timer_callback(self):
        """Controller loop"""

        if self.last_scan is None:
            return # Does not run if the laser message is not received.
        
        ######################## MODIFY CODE HERE ########################
        self.get_logger().debug(str(self.last_scan))
        danger = self.check_danger(self.last_scan, self.angles, self.danger_threshold)

        if np.any(danger):
            self.avoidance_cycles = self.avoidance_hold_cycles
            self.avoidance_angle = self.get_move_angle(self.angles, danger)
        elif self.avoidance_cycles > 0:
            self.avoidance_cycles -= 1

        if self.avoidance_cycles > 0:
            move_angle = self.avoidance_angle
        else:
            move_angle = 0.0

        self.move_2D(
            max_translate_velocity * np.cos(move_angle),
            max_translate_velocity * np.sin(move_angle),
            0.0,
        )
        self.get_logger().debug(
            f"Danger: {np.any(danger)}, Avoidance cycles: {self.avoidance_cycles}, "
            f"Move angle: {move_angle}"
        )

        ######################## MODIFY CODE HERE ########################

    def check_danger(self, scan: np.ndarray, angles: np.ndarray, threshold: float = 0.3) -> np.ndarray:
        """Checks if there is an obstacle within the threshold distance and closer to the target, returns a boolean array of the same shape as scan, where True indicates danger."""
        finite_scan = np.isfinite(scan)
        return finite_scan & (scan < threshold) & (np.abs(angles) < np.pi / 2)

    def get_move_angle(self, angles: np.ndarray, danger: np.ndarray) -> float:
        """Returns the angle to move in to avoid obstacles. Returns 0 if no danger is detected."""
        if not np.any(danger):
            return 0.0

        avg_angle = np.mean(angles[danger])
        perpendicular_angles = np.array([
            avg_angle + np.pi / 2,
            avg_angle - np.pi / 2,
        ])
        direction = perpendicular_angles[np.argmax(np.cos(perpendicular_angles))]
        return np.arctan2(np.sin(direction), np.cos(direction))



def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance_node = ObstacleAvoidanceNode()
    rclpy.spin(obstacle_avoidance_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
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
set_logger_level("obstacle_avoidance", level=LoggingSeverity.DEBUG) 


class LidarScan:
    """A LaserScan with range and angle values paired by array index."""

    def __init__(self, msg: LaserScan, yaw_offset: float = np.pi):
        self.ranges = np.asarray(msg.ranges, dtype=float)
        self.angles = (msg.angle_min + np.arange(self.ranges.size) * msg.angle_increment + yaw_offset) % (2 * np.pi) - np.pi
        self.range_min = msg.range_min
        self.range_max = msg.range_max


class ObstacleAvoidanceNode(Node):
    def __init__(self):
        """Node constructor"""
        super().__init__("obstacle_avoidance")
        self.get_logger().info("Starting Obstacle Avoidance")

        self.pub_cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)  
        self.sub_scan = self.create_subscription(LaserScan, "scan", self.sub_scan_callback, 2) 
        self.last_scan = None 

        self.timer = self.create_timer(0.05, self.timer_callback)  # Runs at 20Hz.

        # --- variables later to be implemented as parameters ---
        self.speed = 0.3  # m/s
        self.fwd_clearance_width = 0.25  # m     the widht that needs to be clear in front of the robot for it to move forward
        self.fwd_clearance_depth = 0.2  # m     the depth that needs to be clear in front of the robot for it to move forward
        self.side_clearance_width = 0.2 # m     the width that needs to be clear on the side of the robot for it to move sideways
        self.side_clearance_depth = 0.1 # m     the depth that needs to be clear on the side of the robot for it to move sideways
        self.side_beam_offset = -0.1     # m     the offset of the side beam from the center of the robot. towards the fron is positive, towards the back is negative.

        # --- FSM Variables ---
        self.state = "FORWARD"
        self.y_deviation = 0.0       # Tracks y coordinate relative to start (+ve is left, -ve is right)
        self.avoid_direction = 1     # 1 for Left, -1 for Right
        
        # --- Debouncing / State Transition Variables ---
        self.confirm_forward_counter = 0
        self.confirm_blocked_counter = 0
        self.CONFIRM_CYCLES = 5      # 5 cycles at 20Hz = 0.25 seconds to verify Lidar reading

    def move_2D(self, x: float = 0.0, y: float = 0.0, turn: float = 0.0):
        """Publishes a twist command to move in 2D space. +ve x is forwards, +ve y is left, and +ve turn is anticlockwise"""
        twist_msg = Twist()
        x = np.clip(x, -max_translate_velocity, max_translate_velocity)
        y = np.clip(y, -max_translate_velocity, max_translate_velocity)
        turn = np.clip(turn, -max_translate_velocity, max_translate_velocity)
        twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z = float(x), float(y), 0.0
        twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z = 0.0, 0.0, float(turn)
        self.pub_cmd_vel.publish(twist_msg)

    def sub_scan_callback(self, msg):
        """Scan subscriber"""
        self.last_scan = LidarScan(msg, yaw_offset=0) 

    def is_path_free(self, scan):
        """Return whether the forward clearance rectangle contains no obstacle."""
        ranges = scan.ranges
        angles = scan.angles
        valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)

        x = ranges[valid] * np.cos(angles[valid])
        y = ranges[valid] * np.sin(angles[valid])
        obstacle_in_rectangle = (
            (x >= 0.0)
            & (x <= self.fwd_clearance_depth)
            & (np.abs(y) <= self.fwd_clearance_width / 2.0)
        )
        return not np.any(obstacle_in_rectangle)

    def is_direction_free(self, scan, direction):
        """Return whether the requested side clearance rectangle is free."""
        if direction not in (-1, 1):
            raise ValueError("direction must be 1 (left) or -1 (right)")

        ranges = scan.ranges
        angles = scan.angles
        valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)

        x = ranges[valid] * np.cos(angles[valid])
        y = ranges[valid] * np.sin(angles[valid])
        x_min = self.side_beam_offset - self.side_clearance_depth / 2.0
        x_max = self.side_beam_offset + self.side_clearance_depth / 2.0
        y_min = min(0.0, direction * self.side_clearance_width)
        y_max = max(0.0, direction * self.side_clearance_width)
        obstacle_in_rectangle = (
            (x >= x_min)
            & (x <= x_max)
            & (y >= y_min)
            & (y <= y_max)
        )
        return not np.any(obstacle_in_rectangle)

    def timer_callback(self):
        """Controller loop"""
        if self.last_scan is None:
            return 

        # Debugging: Print all valid Lidar readings that are less than 1 meter away
        valid_close = (
            np.isfinite(self.last_scan.ranges)
            & (self.last_scan.ranges >= self.last_scan.range_min)
            & (self.last_scan.ranges <= self.last_scan.range_max)
            & (self.last_scan.ranges < 1.0)
        )

        for distance, angle in zip(
            self.last_scan.ranges[valid_close],
            self.last_scan.angles[valid_close]
        ):
            self.get_logger().debug(
                f"Range: {distance:.3f} m, Angle: {np.degrees(angle):.1f}°"
            )


        # State machine debug output
        self.get_logger().debug(
            f"State: {self.state} | Dev: {self.y_deviation:.2f} | "
            f"Fwd Wait: {self.confirm_forward_counter} | Blk Wait: {self.confirm_blocked_counter}"
        )

        # ==========================================
        # STATE 1: MOVE FORWARD
        # ==========================================
        if self.state == "FORWARD":
            if not self.is_path_free(self.last_scan):
                # Stop and wait to confirm obstacle
                self.move_2D(0.0, 0.0, 0.0) 
                self.confirm_blocked_counter += 1
                
                if self.confirm_blocked_counter >= self.CONFIRM_CYCLES:
                    self.state = "AVOID"
                    # Prefer the direction opposite of what we have deviated
                    self.avoid_direction = -1 if self.y_deviation > 0 else 1
                    self.confirm_blocked_counter = 0
            else:
                self.confirm_blocked_counter = 0 # Reset counter if false alarm
                self.move_2D(self.speed, 0.0, 0.0)

        # ==========================================
        # STATE 2: AVOID (LEFT OR RIGHT)
        # ==========================================
        elif self.state == "AVOID":
            # 1. Check if we can go forward again
            if self.is_path_free(self.last_scan):
                self.move_2D(0.0, 0.0, 0.0)
                self.confirm_forward_counter += 1
                self.confirm_blocked_counter = 0 # Reset the other condition's counter
                
                if self.confirm_forward_counter >= self.CONFIRM_CYCLES:
                    self.state = "FORWARD"
                    self.confirm_forward_counter = 0

            # 2. Check if we can't go in our current avoid direction anymore
            elif not self.is_direction_free(self.last_scan, self.avoid_direction):
                self.move_2D(0.0, 0.0, 0.0)
                self.confirm_blocked_counter += 1
                self.confirm_forward_counter = 0 # Reset the other condition's counter
                
                if self.confirm_blocked_counter >= self.CONFIRM_CYCLES:
                    self.state = "AVOID_OPPOSITE"
                    self.avoid_direction *= -1 # Switch to the other side
                    self.confirm_blocked_counter = 0

            # 3. Safe to keep avoiding
            else:
                self.confirm_forward_counter = 0
                self.confirm_blocked_counter = 0
                y_vel = self.speed * self.avoid_direction
                self.move_2D(0.0, y_vel, 0.0)
                self.y_deviation += y_vel * 0.05 # Update deviation (velocity * dt)

        # ==========================================
        # STATE 3: AVOID OPPOSITE DIRECTION
        # ==========================================
        elif self.state == "AVOID_OPPOSITE":
            # 1. Check if we can go forward again
            if self.is_path_free(self.last_scan):
                self.move_2D(0.0, 0.0, 0.0)
                self.confirm_forward_counter += 1
                self.confirm_blocked_counter = 0 # Reset the other condition's counter
                
                if self.confirm_forward_counter >= self.CONFIRM_CYCLES:
                    self.state = "FORWARD"
                    self.confirm_forward_counter = 0

            # 2. Check if we are totally blocked (can't go forward, can't go left, can't go right)
            elif not self.is_direction_free(self.last_scan, self.avoid_direction):
                self.move_2D(0.0, 0.0, 0.0)
                self.confirm_blocked_counter += 1
                self.confirm_forward_counter = 0 # Reset the other condition's counter
                
                if self.confirm_blocked_counter >= self.CONFIRM_CYCLES:
                    self.state = "STOP"
                    self.confirm_blocked_counter = 0
                    self.get_logger().error("Totally blocked! Entering STOP state.")

            # 3. Safe to keep avoiding
            else:
                self.confirm_forward_counter = 0
                self.confirm_blocked_counter = 0
                y_vel = self.speed * self.avoid_direction
                self.move_2D(0.0, y_vel, 0.0)
                self.y_deviation += y_vel * 0.05

        # ==========================================
        # STATE 4: STOP (BLOCKED)
        # ==========================================
        elif self.state == "STOP":
            self.move_2D(0.0, 0.0, 0.0)
            # Will not move unless the node is reset


def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance_node = ObstacleAvoidanceNode()
    rclpy.spin(obstacle_avoidance_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
import numpy as np
import math
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
set_logger_level("obstacle_avoidance", level=LoggingSeverity.INFO) # Configure to either LoggingSeverity.INFO or LoggingSeverity.DEBUG  

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        """Node constructor"""
        super().__init__("obstacle_avoidance")
        self.safe_distance = 1.5
        self.get_logger().info("Starting Obstacle Avoidance")

        self.target_x = 0
        self.target_y = 0
        self.direction = 0
        self.reached = True

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
        self.last_scan = np.array(msg.ranges)[::45]# Slices the 721 scan array to return only 36 scans. Feel free to edit

    def goto(self):
        self.get_logger().info(f"Move to {self.target_x}, {self.target_y}")
        if self.target_x and self.target_y:
            if self.target_x > self.target_y:
                x_speed = (max_translate_velocity if self.target_x > max_translate_velocity else self.target_x)
                y_speed = x_speed*self.target_y/self.target_x*self.direction
            else:
                y_speed = (max_translate_velocity if self.target_y > max_translate_velocity else self.target_y)*self.direction
                x_speed = y_speed*self.target_x/self.target_y*self.direction
            self.move_2D(x_speed,y_speed)
            self.target_x -= x_speed
            self.target_y -= y_speed
        else:
            self.reached = True
        

    def timer_callback(self):
        """Controller loop"""

        if self.last_scan is None:
            return # Does not run if the laser message is not received.
        
        ######################## MODIFY CODE HERE ########################
        #self.get_logger().debug(str(self.last_scan))

        #"""
        if self.last_scan[0] > self.safe_distance:
            if self.last_scan[1] > self.safe_distance:
                if self.last_scan[15] > self.safe_distance:
                    self.move_2D(0.2, 0.0, 0.0)
                    self.get_logger().info("Moving forward.")
                else:
                    self.move_2D(0.1,0.3)
                    self.get_logger().info("Slightly left.")
            else:
                self.move_2D(0.1,-0.3)
                self.get_logger().info("Slightly right.")
        else:
            if self.last_scan[3] > self.last_scan[13]:
                self.move_2D(0, 0.4)
                self.get_logger().info("Left.")
            else:
                self.move_2D(0, -0.4)
                self.get_logger().info("Right.")
        """
        if not self.reached:
            self.goto()
        elif self.last_scan[0] > self.safe_distance and self.last_scan[50] > self.safe_distance and self.last_scan[669] > self.safe_distance:
            self.get_logger().info("Moving forward.")
            self.move_2D(0.2, 0.0, 0.0)
        else:
            for i in range(4,5):
                angle_1 = i*22
                length_1 = self.safe_distance/math.cos(deg_to_rad(angle_1))
                #length_2 = math.sqrt(self.safe_distance**2 + length_1**2 - 2*self.safe_distance*length_1*math.sin(deg_to_rad(angle_1)))
                #angle_2 = angle_1 - rad_to_deg(math.acos((self.safe_distance**2 + length_1**2 - length_2**2)/(2*self.safe_distance*length_1)))

                #print(f"angle 1 = {angle_1}, angle 2 = {angle_2}")
                if self.last_scan[int(angle_1*2)] > length_1: #and self.last_scan[int(angle_2*2)] > length_2:
                    self.reached = False
                    self.target_x = self.safe_distance
                    self.target_y = length_1*math.sin(angle_1)*1.5
                    self.direction = 1
                    break
                elif self.last_scan[int(719-angle_1*2)] > length_1: #and self.last_scan[int(719-angle_2*2)] > length_2:
                    self.reached = False
                    self.target_x = self.safe_distance
                    self.target_y = -length_1*math.sin(angle_1)*1.5
                    self.direction = -1
                    break
            self.goto()
        """
        ######################## MODIFY CODE HERE ########################

def deg_to_rad(deg):
    return deg*math.pi/180

def rad_to_deg(rad):
    return rad*180/math.pi



def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance_node = ObstacleAvoidanceNode()
    rclpy.spin(obstacle_avoidance_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
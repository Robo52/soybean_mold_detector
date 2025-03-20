#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry
import math
import numpy as np
from tf2_ros import Buffer, TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

class NavigationNode(Node):
    """
    Navigation node for controlling the Unitree GO2 robot.
    
    This node handles the robot's movement, interpreting high-level navigation
    commands and translating them into velocity commands for the robot.
    """
    def __init__(self):
        super().__init__('navigation_node')
        
        # Declare parameters
        self.declare_parameter('approach_distance', 0.7)  # Distance to plant (meters)
        self.declare_parameter('angle_tolerance', 0.1)    # Radians
        self.declare_parameter('position_tolerance', 0.1) # Meters
        self.declare_parameter('scan_angular_speed', 0.3) # Rad/s
        
        # Get parameters
        self.approach_distance = self.get_parameter('approach_distance').value
        self.angle_tolerance = self.get_parameter('angle_tolerance').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.scan_angular_speed = self.get_parameter('scan_angular_speed').value
        
        # Initialize robot position and orientation
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        
        # Plant position (to be updated)
        self.plant_x = None
        self.plant_y = None
        
        # Navigation parameters
        self.target_x = None
        self.target_y = None
        self.target_yaw = None
        
        # State variables
        self.scanning = False
        self.moving_to_target = False
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.navigation_status_pub = self.create_publisher(String, 'navigation_status', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        self.navigation_command_sub = self.create_subscription(
            String, 'navigation_commands', self.navigation_command_callback, 10)
        
        # Timer for navigation control loop (10Hz)
        self.create_timer(0.1, self.navigation_loop)
        
        # TF buffer for coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        
        self.get_logger().info("Navigation node initialized")
        
    def odom_callback(self, msg):
        """
        Update robot position from odometry.
        
        Args:
            msg (Odometry): Odometry message containing robot pose
        """
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        # Convert quaternion to Euler angles
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
        
    def navigation_command_callback(self, msg):
        """
        Process navigation commands.
        
        Args:
            msg (String): Navigation command message
        """
        command = msg.data.split()
        
        if command[0] == "START_SCANNING":
            self.get_logger().info("Starting scanning for plants")
            self.scanning = True
            self.moving_to_target = False
            
        elif command[0] == "MOVE_TO_PLANT":
            self.get_logger().info(f"Moving to plant at ({command[1]}, {command[2]})")
            self.scanning = False
            self.moving_to_target = True
            
            # Store plant position
            self.plant_x = float(command[1])
            self.plant_y = float(command[2])
            
            # Calculate optimal position to approach plant
            self.calculate_optimal_position()
            
            # Broadcast plant frame
            self.broadcast_plant_transform()
    
    def broadcast_plant_transform(self):
        """
        Broadcast plant position as a TF frame.
        
        This helps in visualizing the plant location in tools like RViz
        and enables coordinate transformations.
        """
        if self.plant_x is not None and self.plant_y is not None:
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = "map"
            transform.child_frame_id = "plant"
            transform.transform.translation.x = self.plant_x
            transform.transform.translation.y = self.plant_y
            transform.transform.translation.z = 0.0
            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = 0.0
            transform.transform.rotation.z = 0.0
            transform.transform.rotation.w = 1.0
            
            self.tf_broadcaster.sendTransform(transform)
    
    def calculate_optimal_position(self):
        """
        Calculate optimal position for the robot to photograph the plant.
        
        This positions the robot at the appropriate distance from the plant,
        facing the plant, to allow the arm to reach around it for all photos.
        """
        if self.plant_x is None or self.plant_y is None:
            self.get_logger().error("Cannot calculate approach position: plant position is unknown")
            return
            
        # Calculate vector from robot to plant
        dx = self.plant_x - self.robot_x
        dy = self.plant_y - self.robot_y
        
        # Calculate distance and angle to plant
        distance = math.sqrt(dx*dx + dy*dy)
        angle = math.atan2(dy, dx)
        
        # Calculate target position at approach_distance from plant
        self.target_x = self.plant_x - self.approach_distance * math.cos(angle)
        self.target_y = self.plant_y - self.approach_distance * math.sin(angle)
        self.target_yaw = angle  # Face the plant
        
        self.get_logger().info(f"Optimal position: ({self.target_x}, {self.target_y}, {self.target_yaw})")
    
    def navigation_loop(self):
        """
        Main navigation control loop.
        
        This function runs periodically to update robot movement based on
        the current navigation state and goals.
        """
        if self.scanning:
            # Simple scanning behavior: rotate in place
            twist = Twist()
            twist.angular.z = self.scan_angular_speed  # Rotate at configured speed
            self.cmd_vel_pub.publish(twist)
            
        elif self.moving_to_target:
            if self.target_x is None or self.target_y is None:
                self.get_logger().warn("No target position set")
                return
                
            # Calculate distance and angle to target
            dx = self.target_x - self.robot_x
            dy = self.target_y - self.robot_y
            distance = math.sqrt(dx*dx + dy*dy)
            
            # Calculate angle to target
            target_angle = math.atan2(dy, dx)
            angle_diff = self.normalize_angle(target_angle - self.robot_yaw)
            
            # Check if we've reached the target
            if distance < self.position_tolerance:
                # Align to target orientation
                if abs(self.normalize_angle(self.target_yaw - self.robot_yaw)) < self.angle_tolerance:
                    # Target reached and aligned
                    twist = Twist()  # Zero velocity
                    self.cmd_vel_pub.publish(twist)
                    
                    self.moving_to_target = False
                    self.navigation_status_pub.publish(String(data="AT_DESTINATION"))
                    self.get_logger().info("Reached optimal position for photography")
                else:
                    # Align to target orientation
                    twist = Twist()
                    yaw_diff = self.normalize_angle(self.target_yaw - self.robot_yaw)
                    twist.angular.z = 0.5 * yaw_diff  # Proportional control
                    self.cmd_vel_pub.publish(twist)
            else:
                # Move toward target
                twist = Twist()
                
                # If not facing the target, rotate first
                if abs(angle_diff) > 0.3:  # ~17 degrees
                    twist.angular.z = 0.5 * angle_diff  # Proportional control
                else:
                    # Move forward and correct orientation
                    twist.linear.x = 0.3 * min(distance, 0.5)  # Cap speed
                    twist.angular.z = 0.5 * angle_diff  # Proportional control
                    
                self.cmd_vel_pub.publish(twist)
    
    def normalize_angle(self, angle):
        """
        Normalize angle to [-pi, pi].
        
        Args:
            angle (float): Angle in radians
            
        Returns:
            float: Normalized angle in radians
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

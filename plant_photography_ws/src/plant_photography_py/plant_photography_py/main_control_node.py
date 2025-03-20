#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D
import time

class MainControlNode(Node):
    """
    Main control node that orchestrates the plant photography process.
    
    This node acts as the central coordinator for the entire system, managing
    the state machine that guides the robot through plant detection, navigation,
    and photography process.
    """
    def __init__(self):
        super().__init__('main_control_node')
        
        # State variables
        self.plant_detected = False
        self.plant_position = None
        self.current_photo_index = 0
        self.photo_positions = ['front', 'top', 'right_side', 'back', 'left_side']
        self.mission_complete = False
        
        # Publishers
        self.navigation_command_pub = self.create_publisher(
            String, 'navigation_commands', 10)
        self.arm_command_pub = self.create_publisher(
            String, 'arm_commands', 10)
        self.camera_command_pub = self.create_publisher(
            String, 'camera_commands', 10)
        
        # Subscribers
        self.plant_detection_sub = self.create_subscription(
            Detection2D, 'plant_detection', self.plant_detection_callback, 10)
        self.navigation_status_sub = self.create_subscription(
            String, 'navigation_status', self.navigation_status_callback, 10)
        self.arm_status_sub = self.create_subscription(
            String, 'arm_status', self.arm_status_callback, 10)
        self.camera_status_sub = self.create_subscription(
            String, 'camera_status', self.camera_status_callback, 10)
        
        # Initialize the state machine
        self.get_logger().info("Starting plant photography mission")
        self.current_state = "INIT"
        self.create_timer(1.0, self.state_machine_callback)
    
    def plant_detection_callback(self, msg):
        """
        Callback for plant detection messages.
        
        Updates the plant detection state when a plant is identified in the image.
        
        Args:
            msg (Detection2D): Detection message containing plant location
        """
        if not self.plant_detected and msg.results:
            self.get_logger().info("Plant detected!")
            self.plant_detected = True
            # Store the detected plant position
            self.plant_position = msg.results[0].bbox.center
            
    def navigation_status_callback(self, msg):
        """
        Callback for navigation status messages.
        
        Updates the state machine based on the robot's navigation status.
        
        Args:
            msg (String): Navigation status message
        """
        self.get_logger().info(f"Navigation status: {msg.data}")
        if msg.data == "AT_DESTINATION":
            if self.current_state == "MOVING_TO_PLANT":
                self.current_state = "PREPARE_FOR_PHOTOGRAPHY"
                
    def arm_status_callback(self, msg):
        """
        Callback for arm status messages.
        
        Updates the state machine based on the arm's status.
        
        Args:
            msg (String): Arm status message
        """
        self.get_logger().info(f"Arm status: {msg.data}")
        if msg.data == "POSITION_REACHED":
            if self.current_state == "PREPARE_FOR_PHOTOGRAPHY" or self.current_state == "MOVING_ARM":
                self.current_state = "TAKE_PHOTO"
                
    def camera_status_callback(self, msg):
        """
        Callback for camera status messages.
        
        Updates the state machine based on the camera's status.
        
        Args:
            msg (String): Camera status message
        """
        self.get_logger().info(f"Camera status: {msg.data}")
        if msg.data == "PHOTO_TAKEN":
            # Move to the next photo
            self.current_photo_index += 1
            if self.current_photo_index >= len(self.photo_positions):
                self.current_state = "MISSION_COMPLETE"
            else:
                # Move arm to next position
                self.current_state = "MOVING_ARM"
    
    def state_machine_callback(self):
        """
        Main state machine logic.
        
        This function runs periodically to manage the state transitions
        of the robot's photography mission.
        """
        if self.mission_complete:
            return
            
        if self.current_state == "INIT":
            # Initialize robot and begin scanning for plants
            self.get_logger().info("Initializing robot and starting plant detection")
            self.navigation_command_pub.publish(String(data="START_SCANNING"))
            self.current_state = "SCANNING"
            
        elif self.current_state == "SCANNING":
            if self.plant_detected:
                self.get_logger().info("Plant detected, moving to optimal position")
                # Command the robot to move to the plant
                self.navigation_command_pub.publish(
                    String(data=f"MOVE_TO_PLANT {self.plant_position.x} {self.plant_position.y}"))
                self.current_state = "MOVING_TO_PLANT"
                
        elif self.current_state == "MOVING_TO_PLANT":
            # Waiting for navigation_status_callback to update state
            pass
            
        elif self.current_state == "PREPARE_FOR_PHOTOGRAPHY":
            self.get_logger().info("Preparing to take photos")
            # Position the arm for the first photo (front)
            position = self.photo_positions[self.current_photo_index]
            self.arm_command_pub.publish(String(data=f"POSITION_{position.upper()}"))
            
        elif self.current_state == "MOVING_ARM":
            position = self.photo_positions[self.current_photo_index]
            self.get_logger().info(f"Moving arm to {position} position")
            self.arm_command_pub.publish(String(data=f"POSITION_{position.upper()}"))
            
        elif self.current_state == "TAKE_PHOTO":
            position = self.photo_positions[self.current_photo_index]
            self.get_logger().info(f"Taking photo: {position}")
            # Command camera to take photo
            self.camera_command_pub.publish(String(data=f"TAKE_PHOTO {position}"))
            
        elif self.current_state == "MISSION_COMPLETE":
            self.get_logger().info("Mission complete! All photos taken.")
            self.mission_complete = True
            # Return arm to home position
            self.arm_command_pub.publish(String(data="POSITION_HOME"))

def main(args=None):
    rclpy.init(args=args)
    node = MainControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

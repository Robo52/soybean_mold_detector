#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    """
    Launch file for all Python nodes in the plant photography system.
    
    This launches the main control, plant detection, navigation, and camera control nodes.
    """
    # Get package share directory
    pkg_dir = get_package_share_directory('plant_photography_py')
    
    # Configuration files
    detection_config = os.path.join(pkg_dir, 'config', 'detection_config.yaml')
    navigation_config = os.path.join(pkg_dir, 'config', 'navigation_config.yaml')
    
    # Launch Arguments
    use_test_camera = LaunchConfiguration('use_test_camera')
    use_test_camera_arg = DeclareLaunchArgument(
        'use_test_camera',
        default_value='false',
        description='Use test camera instead of real camera'
    )
    
    return LaunchDescription([
        # Launch arguments
        use_test_camera_arg,
        
        # Main control node
        Node(
            package='plant_photography_py',
            executable='main_control_node',
            name='main_control_node',
            output='screen',
        ),
        
        # Plant detection node - identifies plants in camera images
        Node(
            package='plant_photography_py',
            executable='plant_detection_node',
            name='plant_detection_node',
            output='screen',
            parameters=[detection_config]
        ),
        
        # Navigation node - controls robot movement
        Node(
            package='plant_photography_py',
            executable='navigation_node',
            name='navigation_node',
            output='screen',
            parameters=[navigation_config]
        ),
        
        # Camera control node - interfaces with the camera
        Node(
            package='plant_photography_py',
            executable='camera_control_node',
            name='camera_control_node',
            output='screen',
        ),
    ])

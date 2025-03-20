#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    """
    Launch file for the arm control node.
    
    This launches the C++ arm control node with appropriate parameters.
    """
    # Get package share directory
    pkg_dir = get_package_share_directory('d1_arm_control_cpp')
    
    # Configuration file
    arm_config = os.path.join(pkg_dir, 'config', 'arm_config.yaml')
    
    # Launch Arguments
    device_path = LaunchConfiguration('device_path')
    device_path_arg = DeclareLaunchArgument(
        'device_path',
        default_value='/dev/ttyUSB0',
        description='Device path for D1 arm'
    )
    
    return LaunchDescription([
        # Launch arguments
        device_path_arg,
        
        # Arm control node
        Node(
            package='d1_arm_control_cpp',
            executable='arm_control_node',
            name='arm_control_node',
            output='screen',
            parameters=[
                arm_config,
                {'device_path': device_path}
            ]
        ),
    ])

#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    """
    Main launch file for the complete plant photography system.
    
    This launch file includes both the Python and C++ launch files
    to start the entire system.
    """
    # Get the launch files from other packages
    py_pkg_dir = get_package_share_directory('plant_photography_py')
    cpp_pkg_dir = get_package_share_directory('d1_arm_control_cpp')
    
    # Python nodes launch file
    python_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(py_pkg_dir, 'launch', 'python_nodes.launch.py')
        )
    )
    
    # Arm node launch file
    arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cpp_pkg_dir, 'launch', 'arm_node.launch.py')
        )
    )

    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cpp_pkg_dir, 'launch', 'arm_node.launch.py')
        )
    )
    
    # Launch Arguments
    device_path = LaunchConfiguration('device_path')
    device_path_arg = DeclareLaunchArgument(
        'device_path',
        default_value='/dev/ttyUSB0',
        description='Device path for D1 arm'
    )
    
    use_test_camera = LaunchConfiguration('use_test_camera')
    use_test_camera_arg = DeclareLaunchArgument(
        'use_test_camera',
        default_value='false',
        description='Use test camera instead of real camera'
    )
    
    return LaunchDescription([
        # Launch arguments
        device_path_arg,
        use_test_camera_arg,
        
        # Launch files
        python_launch,
        arm_launch,
        zed_launch,
    ])

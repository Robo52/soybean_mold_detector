#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # ZED wrapper node
    zed_node = Node(
        package='zed_wrapper',
        executable='zed_wrapper_node',
        name='zed_node',
        output='screen',
        parameters=[{
            'general.camera_model': 'zed',
            'general.serial_number': 0,  # 0: Use first available camera
            'general.resolution': 2,  # HD720 mode
            'video.rgb_framerate': 15.0,
            'depth.depth_mode': 1,  # PERFORMANCE mode
            'depth.depth_stabilization': 1,  # 1: Enable stabilization
            'pos_tracking.publish_tf': True,
            'pos_tracking.publish_map_tf': True,
            'object_detection.od_enabled': True,
            'object_detection.model': 0,  # 0: MULTI_CLASS_BOX
            # Add more parameters as needed
        }]
    )

    return LaunchDescription([
        zed_node
    ])

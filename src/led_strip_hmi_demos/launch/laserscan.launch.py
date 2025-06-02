#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():

    demos_share = get_package_share_directory('led_strip_hmi_demos')
    cfg = os.path.join(demos_share, 'config', 'laserscan.yaml')

    
    viz_cmd = Node(
            package='led_strip_hmi_visualization',
            executable='strip_visualizer',
            name='strip_visualizer',
            output='screen',
            parameters=[{'strips_config': cfg}],

        )

    projector_cmd = Node(
            package='led_strip_hmi_projector',
            executable='projector_node',
            name='led_strip_hmi_projector',
            output='screen',
            parameters=[{'strips_config': cfg}],
            remappings=[('/scan','scan')]
        )

    return LaunchDescription([
        viz_cmd,
        projector_cmd
    ])

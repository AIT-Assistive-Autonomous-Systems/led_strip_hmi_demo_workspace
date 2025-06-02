#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('dummy_detection_publisher')
    cfg = os.path.join(pkg_share, 'cfg', 'persons.yml')

    return LaunchDescription([
        Node(
            package='dummy_detection_publisher',
            executable='dummy_publisher',
            name='dummy_publisher',
            output='screen',
            parameters=[{'persons_config': cfg}],
        ),
    ])

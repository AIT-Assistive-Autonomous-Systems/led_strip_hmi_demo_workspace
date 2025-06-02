#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    caripu_share = get_package_share_directory('dummy_detection_publisher')
    caripu_cfg = os.path.join(caripu_share, 'cfg', 'persons.yml')

    dummy_publisher = Node(
        package='dummy_detection_publisher',
        executable='dummy_publisher',
        name='dummy_publisher',
        output='screen',
        parameters=[{'persons_config': caripu_cfg}],
    )


    demos_share = get_package_share_directory('led_strip_hmi_demos')
    cfg = os.path.join(demos_share, 'config', 'complex_strip.yaml')


    viz_cmd = Node(
        package='led_strip_hmi_visualization',
        executable='strip_visualizer',
        name='strip_visualizer',
        output='screen',
        parameters=[{'strips_config': cfg}],

    )

#     # Uncomment the following lines to include RViz
    pkg_share = get_package_share_directory('led_strip_hmi_demos')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'default_config.rviz')
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    projector_cmd = Node(
        package='led_strip_hmi_projector',
        executable='projector_node',
        name='led_strip_hmi_projector',
        output='screen',
        parameters=[{'strips_config': cfg}],
        remappings=[('/scan', 'scan')]
    )

    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        output='screen'
    )

    return LaunchDescription([
        dummy_publisher,
        viz_cmd,
        start_rviz_cmd,
        projector_cmd,
        static_tf_publisher
    ])

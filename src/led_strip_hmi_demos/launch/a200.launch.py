#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():

    dummy_share = get_package_share_directory('dummy_detection_publisher')
    dummy_cfg_default = os.path.join(dummy_share, 'cfg', 'person_simple.yml')
    dummy_cfg = DeclareLaunchArgument(
        'dummy', default_value=dummy_cfg_default, description="dummy configuration")

    dummy_publisher = Node(
        package='dummy_detection_publisher',
        executable='dummy_publisher',
        name='dummy_publisher',
        output='screen',
        parameters=[{'persons_config': LaunchConfiguration('dummy')}],
    )

    demos_share = get_package_share_directory('led_strip_hmi_demos')
    strip_cfg = os.path.join(demos_share, 'config', 'a200.yaml')

    viz_cmd = Node(
        package='led_strip_hmi_visualization',
        executable='strip_visualizer',
        name='strip_visualizer',
        output='screen',
        parameters=[{'strips_config': strip_cfg}],

    )

    projector_cmd = Node(
        package='led_strip_hmi_projector',
        executable='projector_node',
        name='led_strip_hmi_projector',
        output='screen',
        parameters=[{'strips_config': strip_cfg}],
        remappings=[('/scan', 'scan')]
    )

    # a200 model and rviz
    rviz_config_file = os.path.join(demos_share, 'rviz', 'a200.rviz')
    a200_robot_path = os.path.join(demos_share, 'config')

    clearpath_viz = get_package_share_directory('clearpath_viz')
    clearpath_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(clearpath_viz, 'launch', 'view_model.launch.py')),
        launch_arguments={"setup_path": a200_robot_path,
                          "config": rviz_config_file
                          }.items()
    )

    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        output='screen'
    )

    return LaunchDescription([
        dummy_cfg,
        static_tf_publisher,
        dummy_publisher,
        viz_cmd,
        projector_cmd,
        clearpath_cmd
    ])

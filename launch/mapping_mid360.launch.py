"""FAST_LIO_Relocation ROS2 — MID360 建图 launch"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('fast_lio')

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Whether to start RViz2')

    rviz_cfg_arg = DeclareLaunchArgument(
        'rviz_cfg',
        default_value=os.path.join(share_dir, 'rviz_cfg', 'loam_livox_ros2.rviz'),
        description='RViz2 config file')

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=os.path.join(share_dir, 'config', 'mid360_mapping_ros2.yaml'),
        description='ROS2 parameters yaml (mapping mode)')

    return LaunchDescription([
        rviz_arg,
        rviz_cfg_arg,
        config_arg,
        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            name='laserMapping',
            output='screen',
            parameters=[LaunchConfiguration('config')],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_cfg')],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])

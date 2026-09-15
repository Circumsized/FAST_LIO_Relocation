"""FAST_LIO_Relocation ROS2 — MID360 重定位（先验地图定位）launch"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _make_node(context):
    """构造 laserMapping 节点；仅当 map_file_path 非空时才覆盖 yaml 中的值。"""
    params = [LaunchConfiguration('config').perform(context)]
    map_file_path = LaunchConfiguration('map_file_path').perform(context)
    if map_file_path:
        params.append({'map_file_path': map_file_path})

    return [
        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            name='laserMapping',
            output='screen',
            parameters=params,
        ),
    ]


def generate_launch_description():
    share_dir = get_package_share_directory('fast_lio')

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Whether to start RViz2')

    rviz_cfg_arg = DeclareLaunchArgument(
        'rviz_cfg',
        default_value=os.path.join(share_dir, 'rviz_cfg', 'loam_livox_relocal_ros2.rviz'),
        description='RViz2 config file')

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=os.path.join(share_dir, 'config', 'mid360_relocalization_ros2.yaml'),
        description='ROS2 parameters yaml (localization mode)')

    map_file_arg = DeclareLaunchArgument(
        'map_file_path',
        default_value='',
        description='Absolute path to the prior PCD map (required in localization mode). '
                    'When non-empty, overrides the value in the config yaml.')

    return LaunchDescription([
        rviz_arg,
        rviz_cfg_arg,
        config_arg,
        map_file_arg,
        OpaqueFunction(function=_make_node),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_cfg')],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])

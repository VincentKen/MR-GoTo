#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch_ros.actions import Node


# Note: Although it might be easier to read (less duplication) and less error prone by importing the existing generate_launch_description functions from
# the other launch files, we decided to copy and paste them into this file for simplicity reasons
map = 'cave'


def generate_launch_description():
    stage = generate_stage_node()
    move = generate_move_node()
    laserscan_features = generate_laserscan_features_node()
    pf = generate_pf_node()
    ekf = generate_ekf_node()

    return LaunchDescription([
        *stage,
        *move,
        *laserscan_features,
        *pf,
        *ekf
    ])


def generate_stage_node():
    package = 'stage_ros2'
    directory = get_package_share_directory(package)

    stage_world_arg = DeclareLaunchArgument(
        'world',
        default_value=TextSubstitution(text=map),
        description='World file relative to the project world file, without .world'
    )

    def stage_world_configuration(context):
        file = os.path.join(directory, 'world',
                            context.launch_configurations['world'] + '.world')
        return [SetLaunchConfiguration('world_file', file)]

    stage_world_configuration_arg = OpaqueFunction(
        function=stage_world_configuration)

    return [
        stage_world_arg,
        stage_world_configuration_arg,
        Node(
            package=package,
            executable=package,
            name='stage',
            parameters=[{
                "world_file": [LaunchConfiguration('world_file')]
            }]
        )
    ]


def generate_move_node():
    return [
        Node(
            package='mr_move',
            executable='move',
            remappings=[('/scan', 'base_scan')],
            parameters=[{
                "mode": "wanderer"
            }]
        )
    ]


def generate_laserscan_features_node():
    return [
        Node(
            package='tuw_laserscan_features',
            executable='composed_node',
            remappings=[('/scan', 'base_scan')],
        )
    ]


def generate_pf_node():
    package = 'mr_pf'
    directory = get_package_share_directory(package)

    yaml_file_name = 'particle_filter.yaml'
    params_arg_name = 'particle_filter_params_file'
    map_file_arg_name = 'particle_filter_map_file'

    pf_params_arg = DeclareLaunchArgument(
        params_arg_name,
        default_value=TextSubstitution(text=yaml_file_name),
        description='ROS2 parameters'
    )

    pf_map_file_arg = DeclareLaunchArgument(
        map_file_arg_name,
        default_value=TextSubstitution(text=map),
        description='map image file'
    )

    def pf_configuration(context):
        yaml_file_path = os.path.join(directory, "config", yaml_file_name)
        png_file_path = os.path.join(directory, "config/maps", map + '.png')

        context.launch_configurations[params_arg_name] = yaml_file_path
        context.launch_configurations[map_file_arg_name] = png_file_path

    pf_configuration_arg = OpaqueFunction(function=pf_configuration)

    return [
        pf_map_file_arg,
        pf_params_arg,
        pf_configuration_arg,
        Node(
            package=package,
            executable='pf_node',
            name='pf',
            remappings=[('/scan', 'base_scan')],
            parameters=[
                LaunchConfiguration(params_arg_name),
                {
                    'map_file': LaunchConfiguration(map_file_arg_name)
                }
            ]
        )
    ]


def generate_ekf_node():
    package = 'mr_ekf'
    directory = get_package_share_directory(package)

    return [
        Node(
            package=package,
            executable='ekf_node',
            name='ekf',
            remappings=[('/scan', 'base_scan')],
            parameters=[
                {
                    'map_file': os.path.join(directory, "config/maps", map + '.png'),
                    'map_linesegments_file': os.path.join(directory, "config/maps", map + '.yml')
                }
            ]
        )
    ]
#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution, PythonExpression
from launch.actions import GroupAction, DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


map = 'cave'


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('localization'),
        generate_stage(),
        generate_laserscan_features(),
        generate_pf(),
        generate_ekf()
    ])


def generate_stage():
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

    return GroupAction([
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
    ])


def generate_laserscan_features():
    return Node(
        package='tuw_laserscan_features',
        executable='composed_node',
        remappings=[('/scan', 'base_scan')],
    )


def generate_pf():
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

    def pf_configuration(context):
        yaml_file_path = os.path.join(directory, "config", yaml_file_name)
        png_file_path = os.path.join(directory, "config/maps", map + '.png')

        context.launch_configurations[params_arg_name] = yaml_file_path
        context.launch_configurations[map_file_arg_name] = png_file_path

    pf_configuration_arg = OpaqueFunction(function=pf_configuration)

    return GroupAction([
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
            ],
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('localization'), "' == 'pf'"]))
        )
    ])


def generate_ekf():
    package = 'mr_ekf'
    directory = get_package_share_directory(package)

    return Node(
        package=package,
        executable='ekf_node',
        name='ekf',
        remappings=[('/scan', 'base_scan')],
        parameters=[
            {
                'map_file': os.path.join(directory, "config/maps", map + '.png'),
                'map_linesegments_file': os.path.join(directory, "config/maps", map + '.yml')
            }
        ],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('localization'), "' == 'ekf'"]))
    )

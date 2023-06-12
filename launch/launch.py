#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import yaml

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution, PythonExpression
from launch.actions import GroupAction, DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


# The params that are available in this launch script
localization_arg_name = 'localization'
map_arg_name = 'map'
params_file_arg_name = 'params_file'


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(localization_arg_name, default_value='ekf', description='Localization method (pf or ekf)'),
        DeclareLaunchArgument(map_arg_name, default_value='line', description='The map that is used'),
        DeclareLaunchArgument(params_file_arg_name, default_value='', description='The params file used for storing params'),
        generate_params_file_configuration(),
        generate_stage(),
        generate_laserscan_features(),
        generate_pf(),
        generate_ekf(),
        generate_goto()
    ])


def generate_params_file_configuration():
    def params_file_configuration(context):
        if context.launch_configurations[params_file_arg_name] != '':
            with open(context.launch_configurations[params_file_arg_name], 'r') as file:
                yaml_data = yaml.safe_load(file)
            goto = yaml_data['goto']
            ros__parameters = goto['ros__parameters']
            localization_arg = ros__parameters[localization_arg_name]
            map_arg = ros__parameters[map_arg_name]

            context.launch_configurations[localization_arg_name] = localization_arg
            context.launch_configurations[map_arg_name] = map_arg
    
    
    return OpaqueFunction(function=params_file_configuration)


def generate_stage():
    package = 'stage_ros2'
    directory = get_package_share_directory(package)

    def stage_world_name(context):
        return [SetLaunchConfiguration('world', context.launch_configurations[map_arg_name])]

    stage_world_arg = OpaqueFunction(function=stage_world_name)

    world_file_arg_name = 'world_file'

    def stage_world_configuration(context):
        file = os.path.join(directory, 'world', context.launch_configurations['world'] + '.world')
        return [SetLaunchConfiguration(world_file_arg_name, file)]

    stage_world_configuration_arg = OpaqueFunction(function=stage_world_configuration)

    return GroupAction([
        stage_world_arg,
        stage_world_configuration_arg,
        Node(
            package=package,
            executable=package,
            name='stage',
            parameters=[{
                "world_file": [LaunchConfiguration(world_file_arg_name)]
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
    map_file_launch_arg_name = 'particle_filter_map_file'

    pf_params_arg = DeclareLaunchArgument(
        params_arg_name,
        default_value=TextSubstitution(text=yaml_file_name),
        description='ROS2 parameters'
    )

    def pf_configuration(context):
        yaml_file_path = os.path.join(directory, "config", yaml_file_name)
        png_file_path = os.path.join(directory, "config/maps", context.launch_configurations[map_arg_name] + '.png')

        context.launch_configurations[params_arg_name] = yaml_file_path
        context.launch_configurations[map_file_launch_arg_name] = png_file_path

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
                    'map_file': LaunchConfiguration(map_file_launch_arg_name)
                }
            ],
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration(localization_arg_name), "' == 'pf'"]))
        )
    ])


def generate_ekf():
    package = 'mr_ekf'
    directory = get_package_share_directory(package)

    map_file_launch_arg_name = 'kalman_filter_map_file'
    map_linesegments_file_arg_name = 'kalman_filter_linesegments_file'

    def ekf_configuration(context):
        map_path = os.path.join(directory, "config/maps", context.launch_configurations[map_arg_name])

        context.launch_configurations[map_file_launch_arg_name] = map_path + '.png'
        context.launch_configurations[map_linesegments_file_arg_name] = map_path + '.yml'
    
    ekf_configuration_arg = OpaqueFunction(function=ekf_configuration)

    return GroupAction([
        ekf_configuration_arg,
        Node(
            package=package,
            executable='ekf_node',
            name='ekf',
            remappings=[('/scan', 'base_scan')],
            parameters=[
                {
                    'map_file': LaunchConfiguration(map_file_launch_arg_name),
                    'map_linesegments_file': LaunchConfiguration(map_linesegments_file_arg_name)
                }
            ],
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration(localization_arg_name), "' == 'ekf'"]))
        )
    ])


def generate_goto():
    package = 'mr_goto'
    directory = get_package_share_directory(package)

    map_file_launch_arg_name = 'goto_map_file'

    def goto_configuration(context):
        map_path = os.path.join(directory, "config/world/bitmaps", context.launch_configurations[map_arg_name])
        context.launch_configurations[map_file_launch_arg_name] = map_path + '.png'

    goto_configuration_arg = OpaqueFunction(function=goto_configuration)

    return GroupAction([
        goto_configuration_arg,
        Node(
            package=package,
            executable='mr_goto',
            name='goto',
            parameters=[
                {
                    'map_file': LaunchConfiguration(map_file_launch_arg_name)
                }
            ]
        )
    ])

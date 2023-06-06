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
map = 'line'


def generate_launch_description():
    # TODO: Write a launch file which starts the stage, mr_move, tuw_laserscan_features and mr_ekf or mr_pf.
    goto_package = 'mr_goto'
    goto_directory = get_package_share_directory(goto_package)

    stage = generate_stage_node()
    move = generate_move_node()
    ekf = generate_ekf_node()

    return LaunchDescription([
        *stage,
        #*move,
        *ekf
    ])


def generate_stage_node():
    """
    Generates and returns the node for the stage

    Returns:
    The stage node
    """
    stage_package = 'stage_ros2'
    stage_directory = get_package_share_directory(stage_package)

    stage_world_arg = DeclareLaunchArgument(
        'world',
        default_value=TextSubstitution(text=map),
        description='World file relative to the project world file, without .world'
    )

    def stage_world_configuration(context):
        file = os.path.join(stage_directory, 'world', context.launch_configurations['world'] + '.world')
        return [SetLaunchConfiguration('world_file', file)]
    
    stage_world_configuration_arg = OpaqueFunction(function=stage_world_configuration)

    return [
        stage_world_arg,
        stage_world_configuration_arg,
        Node(
            package=stage_package,
            executable=stage_package,
            name='stage',
            parameters=[{
                "world_file": [LaunchConfiguration('world_file')]
            }]
        )
    ]


def generate_move_node():
    """
    TODO: Generates and returns the node for the mr_move

    Returns:
    The mr_move node
    """
    pass


def generate_ekf_node():
    pf_package = 'mr_pf'
    pf_directory = get_package_share_directory(pf_package)
    
    pf_params_arg = DeclareLaunchArgument(
        'particle_filter_params_file',
        default_value=TextSubstitution(text='particle_filter.yaml'),
        description='ROS2 parameters'
    )
    
    pf_map_file_arg = DeclareLaunchArgument(
        'particle_filter_map_file',
        default_value=TextSubstitution(text=map),
        description='map image file'
    )

    def pf_configuration(context):
        yaml_file_name = 'particle_filter.yaml'

        yaml_file_path = os.path.join(pf_directory, "config", yaml_file_name)
        png_file_path = os.path.join(pf_directory, "config/maps", map + '.png')

        context.launch_configurations['particle_filter_params_file'] = yaml_file_path
        context.launch_configurations['particle_filter_map_file'] = png_file_path

    pf_configuration_arg = OpaqueFunction(function=pf_configuration)

    return [
        pf_map_file_arg,
        pf_params_arg,
        pf_configuration_arg,
        Node(
            package=pf_package,
            executable='pf_node',
            name='pf',
            remappings=[('/scan', 'base_scan')],
            parameters=[
                LaunchConfiguration('particle_filter_params_file'),
                {
                    'map_file': LaunchConfiguration('particle_filter_map_file')
                }
            ]
        )
    ]

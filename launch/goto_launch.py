#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # TODO: Write a launch file which starts the stage, mr_move, tuw_laserscan_features and mr_ekf or mr_pf.
    this_pgk = 'mr_goto'
    this_pgk_dir = get_package_share_directory(this_pgk)

    stage = generate_stage_node()
    move = generate_move_node()

    return LaunchDescription([
        *stage
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
        default_value=TextSubstitution(text='cave'),
        description='World file relative to the project world file, without .world'
    )

    def stage_world_configuration(context):
        file = os.path.join(
            stage_directory,
            'world',
            context.launch_configurations['world'] + '.world')
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

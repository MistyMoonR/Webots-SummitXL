#!/usr/bin/env python

# Copyright 1996-2021 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not imu_linkn a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#

import os
import pathlib
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node
import launch
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    package_dir = get_package_share_directory('simulator')
    world = LaunchConfiguration('world')
    robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'my_robot.urdf')).read_text()

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world])
    )

    webots_ros2_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
        ]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )

    vlp_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'Velodyne_VLP_16'],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='my_world.wbt',
        ),
        webots,
        webots_ros2_driver,
        robot_state_publisher,
        vlp_publisher,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/Summit_XL_Steel/Velodyne_VLP_16/point_cloud'),
                        ('scan', '/scan')],
            parameters=[{
                'target_frame': 'Velodyne_VLP_16',
                'transform_tolerance': 0.01,
                'min_height': -0.52,
                'max_height': 0.5,
                'angle_min': -3.1415926,  # -M_PI/2
                'angle_max': 3.1415926,  # M_PI/2
                'angle_increment': 0.00349066,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.3,
                'range_max': 20.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        )
    ])

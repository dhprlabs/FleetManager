#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # =========================================================
    # PACKAGE PATHS
    # =========================================================

    pkg_share = get_package_share_directory('virtual_lab')

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_dir = os.path.join(
        nav2_bringup_dir,
        'launch'
    )

    # =========================================================
    # FILES
    # =========================================================

    map_file = os.path.join(
        pkg_share,
        'maps',
        'logistics_warehouse.yaml'
    )

    params_file = os.path.join(
        pkg_share,
        'config',
        'navigation.yaml'
    )

    rviz_config = os.path.join(
        pkg_share,
        'rviz',
        'multi_robot_navigation.rviz'
    )

    # =========================================================
    # LAUNCH ARGUMENTS
    # =========================================================

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    map_yaml = LaunchConfiguration('map')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true'
    )

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true'
    )

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=map_file
    )

    # =========================================================
    # SINGLE LOCALIZATION
    #
    # map_server + AMCL are started ONLY ONCE.
    # =========================================================

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav2_launch_dir,
                'localization_launch.py'
            )
        ),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
        }.items()
    )

    # =========================================================
    # ROBOT 1 NAVIGATION
    #
    # No localization_launch.py here.
    # =========================================================

    robot1_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav2_launch_dir,
                'navigation_launch.py'
            )
        ),
        launch_arguments={
            'namespace': 'robot1',
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
        }.items()
    )

    # =========================================================
    # ROBOT 2 NAVIGATION
    # =========================================================

    robot2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav2_launch_dir,
                'navigation_launch.py'
            )
        ),
        launch_arguments={
            'namespace': 'robot2',
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
        }.items()
    )

    # =========================================================
    # ROBOT 3 NAVIGATION
    # =========================================================

    robot3_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav2_launch_dir,
                'navigation_launch.py'
            )
        ),
        launch_arguments={
            'namespace': 'robot3',
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
        }.items()
    )

    # =========================================================
    # SINGLE RVIZ
    # =========================================================

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d',
            rviz_config
        ],
        parameters=[
            {
                'use_sim_time': use_sim_time
            }
        ]
    )

    # =========================================================
    # LAUNCH DESCRIPTION
    # =========================================================

    return LaunchDescription([

        declare_use_sim_time,
        declare_autostart,
        declare_map,

        # One localization
        localization,

        # Three navigation stacks
        robot1_navigation,
        robot2_navigation,
        robot3_navigation,

        # One RViz
        rviz,
    ])

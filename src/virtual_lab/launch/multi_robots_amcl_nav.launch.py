#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    pkg_dir = get_package_share_directory('virtual_lab')

    config_dir = os.path.join(pkg_dir, 'config')

    map_file = os.path.join(
        pkg_dir,
        'maps',
        'warehouse_map.yaml'
    )

    navigation_params = os.path.join(
        config_dir,
        'navigation.yaml'
    )

    # ============================================================
    # MAP SERVER
    # ============================================================

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'yaml_filename': map_file
            }
        ]
    )

    # ============================================================
    # AMCL - ROBOT 1
    # ============================================================

    amcl_robot1 = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace='robot1',
        output='screen',
        parameters=[
            {
                'use_sim_time': True
            }
        ],
        remappings=[
            ('scan', '/robot1/scan')
        ]
    )

    # ============================================================
    # AMCL - ROBOT 2
    # ============================================================

    amcl_robot2 = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace='robot2',
        output='screen',
        parameters=[
            {
                'use_sim_time': True
            }
        ],
        remappings=[
            ('scan', '/robot2/scan')
        ]
    )

    # ============================================================
    # AMCL - ROBOT 3
    # ============================================================

    amcl_robot3 = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace='robot3',
        output='screen',
        parameters=[
            {
                'use_sim_time': True
            }
        ],
        remappings=[
            ('scan', '/robot3/scan')
        ]
    )

    # ============================================================
    # NAV2 - ROBOT 1
    # ============================================================

    nav2_robot1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            )
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'true',
            'params_file': navigation_params,
            'namespace': 'robot1',
            'use_namespace': 'true',
            'autostart': 'true'
        }.items()
    )

    # ============================================================
    # NAV2 - ROBOT 2
    # ============================================================

    nav2_robot2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            )
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'true',
            'params_file': navigation_params,
            'namespace': 'robot2',
            'use_namespace': 'true',
            'autostart': 'true'
        }.items()
    )

    # ============================================================
    # NAV2 - ROBOT 3
    # ============================================================

    nav2_robot3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            )
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'true',
            'params_file': navigation_params,
            'namespace': 'robot3',
            'use_namespace': 'true',
            'autostart': 'true'
        }.items()
    )

    # ============================================================
    # LAUNCH
    # ============================================================

    return LaunchDescription([

        # Map
        map_server,

        # Localization
        amcl_robot1,
        amcl_robot2,
        amcl_robot3,

        # Navigation
        nav2_robot1,
        nav2_robot2,
        nav2_robot3,
    ])

#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # ============================================================
    # PACKAGE PATH
    # ============================================================

    pkg_virtual_lab = get_package_share_directory('virtual_lab')

    map_file = os.path.join(
        pkg_virtual_lab,
        'maps',
        'logistics_warehouse.yaml'
    )

    amcl_robot1_file = os.path.join(
        pkg_virtual_lab,
        'config',
        'amcl_robot1.yaml'
    )

    amcl_robot2_file = os.path.join(
        pkg_virtual_lab,
        'config',
        'amcl_robot2.yaml'
    )

    amcl_robot3_file = os.path.join(
        pkg_virtual_lab,
        'config',
        'amcl_robot3.yaml'
    )

    # ============================================================
    # SHARED MAP SERVER
    # ============================================================

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        output='screen',

        parameters=[
            {
                'use_sim_time': True,
                'yaml_filename': map_file,
            }
        ]
    )

    # ============================================================
    # ROBOT 1 AMCL
    # ============================================================

    amcl_robot1 = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace='robot1',
        output='screen',

        parameters=[
            amcl_robot1_file,

            # FORCE ROBOT 1 TF FRAME NAMES
            {
                'global_frame_id': 'map',
                'odom_frame_id': 'robot1/odom',
                'base_frame_id': 'robot1/base_footprint',
                'tf_broadcast': True,
                'use_sim_time': True,
            }
        ],

        remappings=[
            ('map', '/map'),
            ('map_updates', '/map_updates'),
        ]
    )

    # ============================================================
    # ROBOT 2 AMCL
    # ============================================================

    amcl_robot2 = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace='robot2',
        output='screen',

        parameters=[
            amcl_robot2_file,

            # FORCE ROBOT 2 TF FRAME NAMES
            {
                'global_frame_id': 'map',
                'odom_frame_id': 'robot2/odom',
                'base_frame_id': 'robot2/base_footprint',
                'tf_broadcast': True,
                'use_sim_time': True,
            }
        ],

        remappings=[
            ('map', '/map'),
            ('map_updates', '/map_updates'),
        ]
    )

    # ============================================================
    # ROBOT 3 AMCL
    # ============================================================

    amcl_robot3 = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace='robot3',
        output='screen',

        parameters=[
            amcl_robot3_file,

            # FORCE ROBOT 3 TF FRAME NAMES
            {
                'global_frame_id': 'map',
                'odom_frame_id': 'robot3/odom',
                'base_frame_id': 'robot3/base_footprint',
                'tf_broadcast': True,
                'use_sim_time': True,
            }
        ],

        remappings=[
            ('map', '/map'),
            ('map_updates', '/map_updates'),
        ]
    )

    # ============================================================
    # LIFECYCLE MANAGER
    # ============================================================

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        namespace='',
        output='screen',

        parameters=[
            {
                'use_sim_time': True,
                'autostart': True,

                'node_names': [
                    'map_server',
                    'robot1/amcl',
                    'robot2/amcl',
                    'robot3/amcl',
                ],
            }
        ]
    )

    # ============================================================
    # LAUNCH
    # ============================================================

    return LaunchDescription([
        map_server,

        amcl_robot1,
        amcl_robot2,
        amcl_robot3,

        lifecycle_manager,
    ])

#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('virtual_lab')

    # ------------------------------------------------------------------
    # Paths
    # ------------------------------------------------------------------

    map_file = os.path.join(
        pkg_share,
        'maps',
        'logistics_warehouse.yaml'
    )

    amcl_robot1_config = os.path.join(
        pkg_share,
        'config',
        'amcl_robot1.yaml'
    )

    amcl_robot2_config = os.path.join(
        pkg_share,
        'config',
        'amcl_robot2.yaml'
    )

    amcl_robot3_config = os.path.join(
        pkg_share,
        'config',
        'amcl_robot3.yaml'
    )

    use_sim_time = LaunchConfiguration(
        'use_sim_time'
    )

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo simulation clock'
    )

    # ------------------------------------------------------------------
    # Map Server
    #
    # ONE map server for the entire multi-robot system.
    # ------------------------------------------------------------------

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {
                'yaml_filename': map_file,
                'use_sim_time': use_sim_time,
            }
        ],
    )

    # ------------------------------------------------------------------
    # Lifecycle Manager
    #
    # Activates the single map_server.
    # ------------------------------------------------------------------

    map_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='map_lifecycle_manager',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': [
                    'map_server',
                ],
            }
        ],
    )

    # ------------------------------------------------------------------
    # AMCL - Robot 1
    # ------------------------------------------------------------------

    amcl_robot1 = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace='robot1',
        output='screen',
        parameters=[
            amcl_robot1_config,
            {
                'use_sim_time': use_sim_time,
            }
        ],
        remappings=[
            ('scan', '/robot1/scan'),
            ('map', '/map'),
        ],
    )

    # ------------------------------------------------------------------
    # AMCL - Robot 2
    # ------------------------------------------------------------------

    amcl_robot2 = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace='robot2',
        output='screen',
        parameters=[
            amcl_robot2_config,
            {
                'use_sim_time': use_sim_time,
            }
        ],
        remappings=[
            ('scan', '/robot2/scan'),
            ('map', '/map'),
        ],
    )

    # ------------------------------------------------------------------
    # AMCL - Robot 3
    # ------------------------------------------------------------------

    amcl_robot3 = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace='robot3',
        output='screen',
        parameters=[
            amcl_robot3_config,
            {
                'use_sim_time': use_sim_time,
            }
        ],
        remappings=[
            ('scan', '/robot3/scan'),
            ('map', '/map'),
        ],
    )

    # ------------------------------------------------------------------
    # Lifecycle Manager for AMCL
    #
    # Each AMCL is namespaced, so use the fully-qualified node names.
    # ------------------------------------------------------------------

    amcl_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='amcl_lifecycle_manager',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': [
                    '/robot1/amcl',
                    '/robot2/amcl',
                    '/robot3/amcl',
                ],
            }
        ],
    )

    rviz_config = os.path.join(
        get_package_share_directory('virtual_lab'),
        'rviz',
        'multi_robot_navigation.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # ------------------------------------------------------------------
    # Launch
    # ------------------------------------------------------------------

    return LaunchDescription([
        use_sim_time_arg,
        rviz_node,
        map_server,
        map_lifecycle_manager,

        amcl_robot1,
        amcl_robot2,
        amcl_robot3,

        amcl_lifecycle_manager,
    ])

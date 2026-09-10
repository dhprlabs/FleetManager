from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    pkg_share = get_package_share_directory('virtual_lab')

    nav2_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch'
    )

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

    # ---------------------------------------------------------
    # ROBOT 1
    # ---------------------------------------------------------

    robot1_nav = GroupAction([
        PushRosNamespace('robot1'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_launch_dir, 'localization_launch.py')
            ),
            launch_arguments={
                'map': map_yaml,
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'params_file': params_file,
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_launch_dir, 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'params_file': params_file,
            }.items()
        ),
    ])

    # ---------------------------------------------------------
    # ROBOT 2
    # ---------------------------------------------------------

    robot2_nav = GroupAction([
        PushRosNamespace('robot2'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_launch_dir, 'localization_launch.py')
            ),
            launch_arguments={
                'map': map_yaml,
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'params_file': params_file,
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_launch_dir, 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'params_file': params_file,
            }.items()
        ),
    ])

    # ---------------------------------------------------------
    # ROBOT 3
    # ---------------------------------------------------------

    robot3_nav = GroupAction([
        PushRosNamespace('robot3'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_launch_dir, 'localization_launch.py')
            ),
            launch_arguments={
                'map': map_yaml,
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'params_file': params_file,
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_launch_dir, 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'params_file': params_file,
            }.items()
        ),
    ])

    # ---------------------------------------------------------
    # ONE RVIZ
    # ---------------------------------------------------------

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_autostart,
        declare_map,

        robot1_nav,
        robot2_nav,
        robot3_nav,

        rviz,
    ])

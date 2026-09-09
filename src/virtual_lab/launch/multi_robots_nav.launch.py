import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # =========================================================
    # Package
    # =========================================================

    pkg_virtual_lab = get_package_share_directory('virtual_lab')

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # =========================================================
    # Files
    # =========================================================

    localization_launch = os.path.join(
        nav2_bringup_dir,
        'launch',
        'localization_launch.py'
    )

    navigation_launch = os.path.join(
        nav2_bringup_dir,
        'launch',
        'navigation_launch.py'
    )

    map_file = os.path.join(
        pkg_virtual_lab,
        'maps',
        'room_with_cones.yaml'
    )

    localization_params = os.path.join(
        pkg_virtual_lab,
        'config',
        'amcl_localization.yaml'
    )

    navigation_params = os.path.join(
        pkg_virtual_lab,
        'config',
        'navigation.yaml'
    )

    rviz_config = os.path.join(
        pkg_virtual_lab,
        'rviz',
        'multi_robot_navigation.rviz'
    )

    # =========================================================
    # Launch arguments
    # =========================================================

    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use Gazebo simulation clock'
    )

    autostart = DeclareLaunchArgument(
        'autostart',
        default_value='True',
        description='Automatically start Nav2'
    )

    use_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='True',
        description='Launch RViz'
    )

    # =========================================================
    # Robot 1
    # =========================================================

    robot1_localization = GroupAction(
        actions=[

            PushRosNamespace('robot1'),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    localization_launch
                ),
                launch_arguments={
                    'map': map_file,
                    'params_file': localization_params,
                    'use_sim_time': LaunchConfiguration(
                        'use_sim_time'
                    ),
                    'autostart': LaunchConfiguration(
                        'autostart'
                    ),
                    'use_composition': 'False',
                }.items()
            ),
        ]
    )

    robot1_navigation = GroupAction(
        actions=[

            PushRosNamespace('robot1'),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    navigation_launch
                ),
                launch_arguments={
                    'params_file': navigation_params,
                    'use_sim_time': LaunchConfiguration(
                        'use_sim_time'
                    ),
                    'autostart': LaunchConfiguration(
                        'autostart'
                    ),
                    'use_composition': 'False',
                }.items()
            ),
        ]
    )

    # =========================================================
    # Robot 2
    # =========================================================

    robot2_localization = GroupAction(
        actions=[

            PushRosNamespace('robot2'),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    localization_launch
                ),
                launch_arguments={
                    'map': map_file,
                    'params_file': localization_params,
                    'use_sim_time': LaunchConfiguration(
                        'use_sim_time'
                    ),
                    'autostart': LaunchConfiguration(
                        'autostart'
                    ),
                    'use_composition': 'False',
                }.items()
            ),
        ]
    )

    robot2_navigation = GroupAction(
        actions=[

            PushRosNamespace('robot2'),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    navigation_launch
                ),
                launch_arguments={
                    'params_file': navigation_params,
                    'use_sim_time': LaunchConfiguration(
                        'use_sim_time'
                    ),
                    'autostart': LaunchConfiguration(
                        'autostart'
                    ),
                    'use_composition': 'False',
                }.items()
            ),
        ]
    )

    # =========================================================
    # Robot 3
    # =========================================================

    robot3_localization = GroupAction(
        actions=[

            PushRosNamespace('robot3'),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    localization_launch
                ),
                launch_arguments={
                    'map': map_file,
                    'params_file': localization_params,
                    'use_sim_time': LaunchConfiguration(
                        'use_sim_time'
                    ),
                    'autostart': LaunchConfiguration(
                        'autostart'
                    ),
                    'use_composition': 'False',
                }.items()
            ),
        ]
    )

    robot3_navigation = GroupAction(
        actions=[

            PushRosNamespace('robot3'),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    navigation_launch
                ),
                launch_arguments={
                    'params_file': navigation_params,
                    'use_sim_time': LaunchConfiguration(
                        'use_sim_time'
                    ),
                    'autostart': LaunchConfiguration(
                        'autostart'
                    ),
                    'use_composition': 'False',
                }.items()
            ),
        ]
    )

    # =========================================================
    # ONE RViz for all robots
    # =========================================================

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_multi_robot',
        arguments=[
            '-d',
            rviz_config
        ],
        parameters=[
            {
                'use_sim_time': LaunchConfiguration(
                    'use_sim_time'
                )
            }
        ],
        condition=IfCondition(
            LaunchConfiguration('rviz')
        ),
        output='screen'
    )

    # =========================================================
    # Launch everything
    # =========================================================

    return LaunchDescription([

        use_sim_time,
        autostart,
        use_rviz,

        robot1_localization,
        robot1_navigation,

        robot2_localization,
        robot2_navigation,

        robot3_localization,
        robot3_navigation,

        rviz,
    ])

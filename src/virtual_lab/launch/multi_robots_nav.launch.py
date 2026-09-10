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
from launch_ros.actions import ComposableNodeContainer, Node
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
        'logistics_warehouse.yaml'
    )

    localization_params_robot1 = os.path.join(pkg_virtual_lab, 'config', 'amcl_localization_robot1.yaml')
    navigation_params_robot1   = os.path.join(pkg_virtual_lab, 'config', 'navigation_robot1.yaml')

    localization_params_robot2 = os.path.join(pkg_virtual_lab, 'config', 'amcl_localization_robot2.yaml')
    navigation_params_robot2   = os.path.join(pkg_virtual_lab, 'config', 'navigation_robot2.yaml')

    localization_params_robot3 = os.path.join(pkg_virtual_lab, 'config', 'amcl_localization_robot3.yaml')
    navigation_params_robot3   = os.path.join(pkg_virtual_lab, 'config', 'navigation_robot3.yaml')

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

    robot1_container = ComposableNodeContainer(
        package='rclcpp_components',
        executable='component_container_mt',
        name='nav2_container',
        namespace='robot1',
        output='screen',
    )

    robot1_localization = GroupAction(
        actions=[
            PushRosNamespace('robot1'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    localization_launch
                ),
                launch_arguments={
                    'namespace': 'robot1',
                    'map': map_file,
                    'params_file': localization_params_robot1, 
                    'use_sim_time': LaunchConfiguration(
                        'use_sim_time'
                    ),
                    'autostart': LaunchConfiguration(
                        'autostart'
                    ),
                    'use_composition': 'False',
                    'container_name': 'nav2_container',
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
                    'namespace': 'robot1',
                    'params_file': navigation_params_robot1,
                    'use_sim_time': LaunchConfiguration(
                        'use_sim_time'
                    ),
                    'autostart': LaunchConfiguration(
                        'autostart'
                    ),
                    'use_composition': 'False',
                    'container_name': 'nav2_container',
                }.items()
            ),
        ]
    )

    # =========================================================
    # Robot 2
    # =========================================================

    robot2_container = ComposableNodeContainer(
        package='rclcpp_components',
        executable='component_container_mt',
        name='nav2_container',
        namespace='robot2',
        output='screen',
    )

    robot2_localization = GroupAction(
        actions=[
            PushRosNamespace('robot2'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    localization_launch
                ),
                launch_arguments={
                    'namespace': 'robot2',
                    'map': map_file,
                    'params_file': localization_params_robot2,
                    'use_sim_time': LaunchConfiguration(
                        'use_sim_time'
                    ),
                    'autostart': LaunchConfiguration(
                        'autostart'
                    ),
                    'use_composition': 'False',
                    'container_name': 'nav2_container',
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
                    'namespace': 'robot2',
                    'params_file': navigation_params_robot2, 
                    'use_sim_time': LaunchConfiguration(
                        'use_sim_time'
                    ),
                    'autostart': LaunchConfiguration(
                        'autostart'
                    ),
                    'use_composition': 'False',
                    'container_name': 'nav2_container',
                }.items()
            ),
        ]
    )

    # =========================================================
    # Robot 3
    # =========================================================

    robot3_container = ComposableNodeContainer(
        package='rclcpp_components',
        executable='component_container_mt',
        name='nav2_container',
        namespace='robot3',
        output='screen',
    )

    robot3_localization = GroupAction(
        actions=[
            PushRosNamespace('robot3'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    localization_launch
                ),
                launch_arguments={
                    'namespace': 'robot3',
                    'map': map_file,
                    'params_file': localization_params_robot3,
                    'use_sim_time': LaunchConfiguration(
                        'use_sim_time'
                    ),
                    'autostart': LaunchConfiguration(
                        'autostart'
                    ),
                    'use_composition': 'False',
                    'container_name': 'nav2_container',
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
                    'namespace': 'robot3',
                    'params_file': navigation_params_robot3,
                    'use_sim_time': LaunchConfiguration(
                        'use_sim_time'
                    ),
                    'autostart': LaunchConfiguration(
                        'autostart'
                    ),
                    'use_composition': 'False',
                    'container_name': 'nav2_container',
                }.items()
            ),
        ]
    )

    # =========================================================
    # GLOBAL MAP SERVER (serves /map for RViz and fleet)
    # =========================================================

    global_map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {
                'yaml_filename': map_file,
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ]
    )

    lifecycle_manager_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': True,
                'node_names': ['map_server']
            }
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
        additional_env={
            'LD_PRELOAD': '/lib/x86_64-linux-gnu/libpthread.so.0',
        },
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

        global_map_server,
        lifecycle_manager_map,

        robot1_localization,
        robot1_navigation,

        robot2_localization,
        robot2_navigation,

        robot3_localization,
        robot3_navigation,

        rviz,
    ])

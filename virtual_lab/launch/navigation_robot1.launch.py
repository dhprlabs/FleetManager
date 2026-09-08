import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # =========================================================
    # PACKAGES
    # =========================================================

    pkg_virtual_lab = get_package_share_directory('virtual_lab')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # =========================================================
    # FILES
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
        'robot1_navigation.rviz'
    )

    # =========================================================
    # ARGUMENTS
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
    # ROBOT 1 LOCALIZATION
    # =========================================================

    robot1_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            localization_launch
        ),
        launch_arguments={
            'namespace': 'robot1',
            'map': map_file,
            'params_file': localization_params,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'use_composition': 'False',
            'use_respawn': 'False',
        }.items()
    )

    # =========================================================
    # ROBOT 1 NAVIGATION
    # =========================================================

    robot1_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            navigation_launch
        ),
        launch_arguments={
            'namespace': 'robot1',
            'params_file': navigation_params,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'use_composition': 'False',
            'use_respawn': 'False',
        }.items()
    )

    # =========================================================
    # ROBOT 1 RVIZ
    # =========================================================

    robot1_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='robot1_rviz',
        arguments=[
            '-d',
            rviz_config
        ],
        parameters=[
            {
                'use_sim_time':
                    LaunchConfiguration('use_sim_time')
            }
        ],
        condition=IfCondition(
            LaunchConfiguration('rviz')
        ),
        output='screen'
    )

    # =========================================================
    # LAUNCH
    # =========================================================

    return LaunchDescription([

        use_sim_time,
        autostart,
        use_rviz,

        robot1_localization,
        robot1_navigation,

        robot1_rviz,
    ])

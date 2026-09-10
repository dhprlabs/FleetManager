import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


# Default initial positions matching Gazebo multi-robot warehouse spawn
DEFAULT_SPAWN_POSES = {
    'robot1': (-10.7947, 0.5379),
    'robot2': (-9.3141, -3.3217),
    'robot3': (3.9202, -1.6023),
    'robot_1': (-10.7947, 0.5379),
    'robot_2': (-9.3141, -3.3217),
    'robot_3': (3.9202, -1.6023),
}


def launch_setup(context, *args, **kwargs):
    robots_str = LaunchConfiguration('robots').perform(context)
    launch_broadcaster = LaunchConfiguration('launch_broadcaster').perform(context).lower() == 'true'
    auto_generate = LaunchConfiguration('auto_generate_tasks').perform(context).lower() == 'true'
    comm_radius = float(LaunchConfiguration('communication_radius').perform(context))
    dropout_rate = float(LaunchConfiguration('packet_dropout_rate').perform(context))
    dock_x = float(LaunchConfiguration('dock_x').perform(context))
    dock_y = float(LaunchConfiguration('dock_y').perform(context))
    broadcast_radius = float(LaunchConfiguration('broadcast_radius').perform(context))
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'

    robot_list = [r.strip() for r in robots_str.split(',') if r.strip()]
    actions = []

    # 1. Launch Task Broadcaster (at Dock Station)
    if launch_broadcaster:
        actions.append(
            Node(
                package='fleet_manager',
                executable='task_broadcaster',
                name='task_broadcaster',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'auto_generate_sample_tasks': auto_generate,
                    'publish_rate': 1.0,
                    'warehouse_frame': 'map',
                    'dock_x': dock_x,
                    'dock_y': dock_y,
                    'broadcast_radius': broadcast_radius,
                }]
            )
        )

    # 2. Launch Identical Robot Stack
    for robot_name in robot_list:

        robot_nodes = [
            Node(
                package='fleet_manager',
                executable='state_manager',
                name='state_manager',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'robot_id': robot_name,
                    'beacon_rate': 1.0,
                    'update_rate': 2.0,
                }]
            ),
            Node(
                package='fleet_manager',
                executable='task_manager',
                name='task_manager',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time, 'robot_id': robot_name}]
            ),
            Node(
                package='fleet_manager',
                executable='maxsum_allocator',
                name='maxsum_allocator',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time, 'robot_id': robot_name}]
            ),
            Node(
                package='fleet_manager',
                executable='bundle_manager',
                name='bundle_manager',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time, 'robot_id': robot_name}]
            ),
            Node(
                package='fleet_manager',
                executable='intent_broadcaster',
                name='intent_broadcaster',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time, 'robot_id': robot_name}]
            ),
            Node(
                package='fleet_manager',
                executable='pickup_validator',
                name='pickup_validator',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time, 'robot_id': robot_name}]
            ),
            Node(
                package='fleet_manager',
                executable='reservation_manager',
                name='reservation_manager',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time, 'robot_id': robot_name}]
            ),
            Node(
                package='fleet_manager',
                executable='conflict_resolver',
                name='conflict_resolver',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time, 'robot_id': robot_name}]
            ),
            Node(
                package='fleet_manager',
                executable='nav2_bridge',
                name='nav2_bridge',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time, 'robot_id': robot_name}]
            ),
            Node(
                package='fleet_manager',
                executable='p2p_transport',
                name='p2p_transport',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'robot_id': robot_name,
                    'communication_radius': comm_radius,
                    'packet_dropout_rate': dropout_rate,
                    'beacon_rate': 2.0,
                    'peer_timeout': 3.0,
                }]
            ),
        ]

        actions.append(
            GroupAction(
                actions=[
                    PushRosNamespace(robot_name),
                    *robot_nodes
                ]
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robots',
            default_value='robot1,robot2,robot3',
            description='Comma-separated list of robot namespaces'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),
        DeclareLaunchArgument(
            'launch_broadcaster',
            default_value='true',
            description='Whether to launch the central Task Broadcaster'
        ),
        DeclareLaunchArgument(
            'auto_generate_tasks',
            default_value='true',
            description='Whether the broadcaster automatically publishes sample tasks T1-T5'
        ),
        DeclareLaunchArgument(
            'communication_radius',
            default_value='6.0',
            description='Simulated P2P radio range in meters'
        ),
        DeclareLaunchArgument(
            'packet_dropout_rate',
            default_value='0.0',
            description='Simulated packet loss probability (0.0 to 1.0)'
        ),
        DeclareLaunchArgument(
            'dock_x',
            default_value='-9.9734',
            description='X coordinate of dock station / task broadcaster'
        ),
        DeclareLaunchArgument(
            'dock_y',
            default_value='-1.4383',
            description='Y coordinate of dock station / task broadcaster'
        ),
        DeclareLaunchArgument(
            'broadcast_radius',
            default_value='6.0',
            description='Wireless transmission radius of dock station broadcaster'
        ),
        OpaqueFunction(function=launch_setup),
    ])

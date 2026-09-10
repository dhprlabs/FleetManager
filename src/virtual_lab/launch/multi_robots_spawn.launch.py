import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _append_gazebo_resource_path(resource_path):
    for env_name in ("GZ_SIM_RESOURCE_PATH", "IGN_GAZEBO_RESOURCE_PATH"):
        existing_path = os.environ.get(env_name, "")
        os.environ[env_name] = (
            existing_path + os.pathsep + resource_path if existing_path else resource_path
        )


def generate_launch_description():

    pkg_virtual_lab = get_package_share_directory('virtual_lab')

    gazebo_models_path, ignore_last_dir = os.path.split(pkg_virtual_lab)
    _append_gazebo_resource_path(gazebo_models_path)


    # =========================================================
    # ARGUMENTS
    # =========================================================

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='logistics_warehouse.sdf',
        description='Gazebo world file'
    )

    model_arg = DeclareLaunchArgument(
        'model',
        default_value='mogi_bot.urdf.xacro',
        description='Robot xacro file'
    )


    # =========================================================
    # ROBOT DESCRIPTION PATH
    # =========================================================

    urdf_file_path = PathJoinSubstitution([
        pkg_virtual_lab,
        "urdf",
        LaunchConfiguration('model')
    ])


    # =========================================================
    # GAZEBO WORLD
    # =========================================================

    world_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                pkg_virtual_lab,
                'launch',
                'world.launch.xml'
            )
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
        }.items()
    )


    # =========================================================
    # ROBOT 1
    # =========================================================

    robot1_description = Command([
        'xacro ',
        urdf_file_path,
        ' robot_name:=robot1'
    ])

    robot1_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='robot1',
        name='robot_state_publisher',
        output='screen',

        parameters=[
            {
                'robot_description': robot1_description,
                'use_sim_time': True
            }
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )


    robot1_spawn = Node(
        package='ros_gz_sim',
        executable='create',

        arguments=[
            '-name', 'robot1',
            '-string', robot1_description,

            '-x', '4.88',
            '-y', '-6.30',
            '-z', '0.30',
            '-Y', '1.57'
        ],

        output='screen',

        parameters=[
            {
                'use_sim_time': True
            }
        ]
    )


    # =========================================================
    # ROBOT 2
    # =========================================================

    robot2_description = Command([
        'xacro ',
        urdf_file_path,
        ' robot_name:=robot2'
    ])

    robot2_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='robot2',
        name='robot_state_publisher',
        output='screen',

        parameters=[
            {
                'robot_description': robot2_description,
                'use_sim_time': True
            }
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )


    robot2_spawn = Node(
        package='ros_gz_sim',
        executable='create',

        arguments=[
            '-name', 'robot2',
            '-string', robot2_description,

            '-x', '2.0',
            '-y', '-6.30',
            '-z', '0.30',
            '-Y', '1.57'
        ],

        output='screen',

        parameters=[
            {
                'use_sim_time': True
            }
        ]
    )


    # =========================================================
    # ROBOT 3
    # =========================================================

    robot3_description = Command([
        'xacro ',
        urdf_file_path,
        ' robot_name:=robot3'
    ])

    robot3_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='robot3',
        name='robot_state_publisher',
        output='screen',

        parameters=[
            {
                'robot_description': robot3_description,
                'use_sim_time': True
            }
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )


    robot3_spawn = Node(
        package='ros_gz_sim',
        executable='create',

        arguments=[
            '-name', 'robot3',
            '-string', robot3_description,

            '-x', '-3.88',
            '-y', '-6.30',
            '-z', '0.30',
            '-Y', '1.57'
        ],

        output='screen',

        parameters=[
            {
                'use_sim_time': True
            }
        ]
    )


    # =========================================================
    # GAZEBO <-> ROS BRIDGE
    # =========================================================

    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",

        parameters=[
            {
                "config_file": os.path.join(
                    pkg_virtual_lab,
                    "config",
                    "gz_bridge.yaml"
                ),

                "use_sim_time": True,
            }
        ],

        output="screen"
    )


    # =========================================================
    # IMAGE BRIDGE
    # =========================================================

    # gz_image_bridge_node = Node(
    #     package="ros_gz_image",
    #     executable="image_bridge",

    #     arguments=[
    #         "/camera/image"
    #     ],

    #     output="screen",

    #     parameters=[
    #         {
    #             'use_sim_time': True,
    #             'camera.image.compressed.jpeg_quality': 75
    #         }
    #     ]
    # )


    # =========================================================
    # TF RELAY: /robotX/tf  →  /tf  (global TF tree)
    #
    # The gz bridge publishes each robot's odometry TF to the
    # namespaced topic /robotX/tf, but TF2 (and Nav2 / AMCL)
    # only listens on the global /tf topic.  These relay nodes
    # merge all robot TF streams into /tf so the `map` frame
    # can be seen by every Nav2 node.
    # =========================================================

    tf_relay_robot1 = Node(
        package='topic_tools',
        executable='relay',
        name='tf_relay_robot1',
        output='screen',
        arguments=['/robot1/tf', '/tf'],
        parameters=[{'use_sim_time': True}],
    )

    tf_relay_robot2 = Node(
        package='topic_tools',
        executable='relay',
        name='tf_relay_robot2',
        output='screen',
        arguments=['/robot2/tf', '/tf'],
        parameters=[{'use_sim_time': True}],
    )

    tf_relay_robot3 = Node(
        package='topic_tools',
        executable='relay',
        name='tf_relay_robot3',
        output='screen',
        arguments=['/robot3/tf', '/tf'],
        parameters=[{'use_sim_time': True}],
    )


    # =========================================================
    # TF_STATIC RELAY: /robotX/tf_static  →  /tf_static
    # =========================================================

    tf_static_relay_robot1 = Node(
        package='topic_tools',
        executable='relay',
        name='tf_static_relay_robot1',
        output='screen',
        arguments=['/robot1/tf_static', '/tf_static'],
        parameters=[{'use_sim_time': True}],
    )

    tf_static_relay_robot2 = Node(
        package='topic_tools',
        executable='relay',
        name='tf_static_relay_robot2',
        output='screen',
        arguments=['/robot2/tf_static', '/tf_static'],
        parameters=[{'use_sim_time': True}],
    )

    tf_static_relay_robot3 = Node(
        package='topic_tools',
        executable='relay',
        name='tf_static_relay_robot3',
        output='screen',
        arguments=['/robot3/tf_static', '/tf_static'],
        parameters=[{'use_sim_time': True}],
    )


    # =========================================================
    # LAUNCH DESCRIPTION
    # =========================================================

    ld = LaunchDescription()

    ld.add_action(world_arg)
    ld.add_action(model_arg)

    ld.add_action(world_launch)

    # Robot 1
    ld.add_action(robot1_state_publisher)
    ld.add_action(robot1_spawn)

    # Robot 2
    ld.add_action(robot2_state_publisher)
    ld.add_action(robot2_spawn)

    # Robot 3
    ld.add_action(robot3_state_publisher)
    ld.add_action(robot3_spawn)

    # Bridges
    ld.add_action(gz_bridge_node)
    # ld.add_action(gz_image_bridge_node)

    # TF relays (namespaced → global /tf)
    ld.add_action(tf_relay_robot1)
    ld.add_action(tf_relay_robot2)
    ld.add_action(tf_relay_robot3)

    # TF_STATIC relays (namespaced → global /tf_static)
    ld.add_action(tf_static_relay_robot1)
    ld.add_action(tf_static_relay_robot2)
    ld.add_action(tf_static_relay_robot3)

    return ld

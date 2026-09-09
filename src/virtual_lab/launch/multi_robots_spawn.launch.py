import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_virtual_lab = get_package_share_directory('virtual_lab')

    gazebo_models_path, ignore_last_dir = os.path.split(pkg_virtual_lab)
    os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path


    # =========================================================
    # ARGUMENTS
    # =========================================================

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='room_with_cones.sdf',
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
            '-topic', '/robot1/robot_description',

            '-x', '4.88',
            '-y', '-6.30',
            '-z', '0.10',
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
            '-topic', '/robot2/robot_description',

            '-x', '2.0',
            '-y', '-6.30',
            '-z', '0.10',
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
            '-topic', '/robot3/robot_description',

            '-x', '-0.88',
            '-y', '-6.30',
            '-z', '0.10',
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

    gz_image_bridge_node = Node(
        package="ros_gz_image",
        executable="image_bridge",

        arguments=[
            "/camera/image"
        ],

        output="screen",

        parameters=[
            {
                'use_sim_time': True,
                'camera.image.compressed.jpeg_quality': 75
            }
        ]
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
    ld.add_action(gz_image_bridge_node)

    return ld

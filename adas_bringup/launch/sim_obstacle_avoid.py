import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    use_sim_time  = LaunchConfiguration('use_sim_time', default='true')
    sim_share     = get_package_share_directory('sim_world')
    bringup_share = get_package_share_directory('adas_bringup')

    model_path  = os.path.join(sim_share, 'models')
    world_file  = os.path.join(sim_share, 'worlds', 's_track_world.sdf')
    bridge_cfg  = os.path.join(bringup_share, 'config', 'bridge.yaml')
    yolo_params = os.path.join(
        get_package_share_directory('obstacle_handler'),
        'config', 'yolo_params.yaml'
    )

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    return LaunchDescription([

        #  Ignition resource path 
        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=model_path
        ),

        #  1. Gazebo sim 
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': [world_file, ' -r']}.items(),
        ),

        #  2. Spawn ackermann_bot ─
        Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=[
                '-entity', 'ackermann_bot',
                '-name',   'ackermann_bot',
                '-file', PathJoinSubstitution([
                    get_package_share_directory('sim_world'),
                    'models', 'ackermann_bot', 'model.sdf'
                ]),
                '-allow_renaming', 'true',
                '-x', '0.0', '-y', '0.0', '-z', '0.01'
            ]
        ),

        #  3. ROS↔Gazebo bridge ─
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['--ros-args', '-p', f'config_file:={bridge_cfg}'],
            output='screen',
            name='gz_bridge'
        ),

        #  4. Lane detector ─
        # IMPORTANT: remap /cmd_vel → /lane_cmd_vel so it does NOT drive the
        # robot directly. The obstacle_handler arbitrates and forwards to
        # /cmd_vel which the Ackermann plugin is listening to.
        Node(
            package='perception_handler',
            executable='temporal_lane_detect',
            output='screen',
            name='lane_detector',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ('/cmd_vel', '/lane_cmd_vel'),
            ]
        ),

        #  5. Obstacle handler 
        # Subscribes to /camera/image_raw and /lane_cmd_vel.
        # Publishes final arbitrated command to /cmd_vel → Ackermann plugin.
        TimerAction(
            period=3.0,   # small delay so Gazebo and bridge are up first
            actions=[
                Node(
                    package='obstacle_handler',
                    executable='obstacle_handler_node',
                    output='screen',
                    name='obstacle_handler',
                    parameters=[
                        yolo_params,
                        {'use_sim_time': use_sim_time}
                    ]
                )
            ]
        ),

    ])
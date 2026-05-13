import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
 
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    sim_share   = get_package_share_directory('sim_world')
    bringup_share = get_package_share_directory('adas_bringup')
 
    # Tell Ignition where to find the model
    model_path = os.path.join(sim_share, 'models')
    world_file = os.path.join(sim_share, 'worlds', 'lane_world.sdf')
    bridge_cfg  = os.path.join(bringup_share, 'config', 'bridge.yaml')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

 
    return LaunchDescription([
 
        # Set IGN_GAZEBO_RESOURCE_PATH so Ignition finds ackermann_bot model
        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=model_path
        ),


        # 1. Include Gazebo Launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': [world_file,' -r']}.items(),
        ),        

        Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=[
                '-entity', 'ackermann_bot',
                '-name', 'ackermann_bot',
                '-file', PathJoinSubstitution([
                    get_package_share_directory('sim_world'),
                    "models", "ackermann_bot", "model.sdf"
                ]),
                '-allow_renaming', 'true',
                '-x', '-0.0',
                '-y', '-0.0',
                '-z', '0.01'
            ]
        ),
 
        # ros_gz_bridge — connects Ignition topics to ROS2
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['--ros-args', '-p',
                       f'config_file:={bridge_cfg}'],
            output='screen',
            name='gz_bridge'
        ),
 
        # Lane detection perception node
        # Node(
        #     package='criticise_perception',
        #     executable='lane_detection_node',
        #     output='screen',
        #     name='lane_detection',
        #     parameters=[{
        #         'image_topic': '/camera/image_raw',
        #         'debug_view': True,
        #     }]
        # ),

        # TimerAction(
        #     period=5.0,
        #     actions=[
        #         Node(
        #             package='rviz2',
        #             executable='rviz2',
        #             name='rviz2',
        #             arguments=['-d', rviz_config_dir],
        #             parameters=[{'use_sim_time': use_sim_time}],
        #             output='screen'
        #         )
        #     ]
        # )
    ])

"""Launch world with panda and several objectes to demonstrate throwing ability"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    world = os.path.join(get_package_share_directory('ign_moveit2'),
                         'worlds', 'panda_place.sdf')

    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description="If true, use simulated clock"),

        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_ign_gazebo'),
                              'launch', 'ign_gazebo.launch.py')]),
            launch_arguments=[('ign_args', [world, ' -r'])]),

        # Pose of the throwing object (IGN -> ROS2)
        Node(package='ros_ign_bridge',
             executable='parameter_bridge',
             name='parameter_bridge_throwing_object_pose',
             output='screen',
             arguments=[
                 '/model/throwing_object/pose@geometry_msgs/msg/Pose[ignition.msgs.Pose'],
             parameters=[{'use_sim_time': use_sim_time}]),
    ])

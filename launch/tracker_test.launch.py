from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction
import os

def generate_launch_description():
    return LaunchDescription([
        # Launch RViz
        ExecuteProcess(
            cmd=[
                'rviz2', '-d',
                '/home/user/aiil_workspace/humble_workspace/src/aiil_rosbot_demo/rviz/gazebo.rviz'
            ],
            output='screen'
        ),

        # Launch Your Tracking Node
       TimerAction(
            period=10.0,  # Wait 10 seconds after everything else
            actions=[
                Node(
                    package='rosbot_navigation',
                    executable='position_tracker_node',
                    name='position_tracker',
                    output='screen'
                )
            ]
        )
    ])

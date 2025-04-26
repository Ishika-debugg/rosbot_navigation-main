# launch/navigation.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the package
    pkg_dir = get_package_share_directory('rosbot_navigation')
    
    # Path to the parameter file
    params_file = os.path.join(pkg_dir, 'config', 'navigation_params.yaml')
    
    # Define the launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    # Define the node
    navigation_node = Node(
        package='rosbot_navigation',
        executable='navigation_logic',
        name='navigation_logic',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # Create the launch description and add actions
    ld = LaunchDescription()
    
    # Add the declared launch arguments
    ld.add_action(declare_use_sim_time)
    
    # Add the node
    ld.add_action(navigation_node)
    
    return ld

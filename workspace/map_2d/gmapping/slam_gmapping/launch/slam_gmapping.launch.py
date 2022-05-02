import os

from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable
import launch.actions
import launch_ros.actions
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)

def generate_launch_description():
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='false')
    ekf_dir = get_package_share_directory('kalman_filter_localization')
    ekf_launch_dir = os.path.join(ekf_dir, 'launch')

    return LaunchDescription([
        launch_ros.actions.Node(
            package='slam_gmapping', node_executable='slam_gmapping', output='screen', 
            parameters=[{'use_sim_time':use_sim_time}]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ekf_launch_dir, 'ekf.launch.py')),
        ),        
    ])
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

  # Set the path to different files and folders.  
  pkg_share = FindPackageShare(package='mobile_robot_ekf').find('mobile_robot_ekf')
  default_launch_dir = os.path.join(pkg_share, 'launch')
  # robot_localization_file_path = os.path.join(pkg_share, 'config/ekf_with_gps.yaml')
  robot_localization_file_path = '/home/sp/ws/src/COMP390-webots/vx_ws/src/robot-pose-ekf-master/config/ekf_with_gps.yaml'
  print(robot_localization_file_path)
  
  # Launch configuration variables specific to simulation
  use_rviz = LaunchConfiguration('use_rviz')

  # Declare the launch arguments  
  declare_use_rviz_cmd = DeclareLaunchArgument(
    name='use_rviz',
    default_value='True',
    description='Whether to start RVIZ')

  # Specify the actions
  # Start robot localization using an Extended Kalman filter
  start_gps_point_to_pose_cmd = Node(
    package='mobile_robot_ekf',
    executable='gps_point_to_pose',
    name='gps_point_to_pose',
    output='log')
    
  # Start robot localization using an Extended Kalman filter...odom->base_link transform
  start_robot_localization_local_cmd = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node_odom',
    output='screen',
    parameters=[robot_localization_file_path],
    remappings=[('odometry/filtered', '/odometry/ekf'),
                ('/set_pose', '/initialpose')])


  # Launch RViz
  start_rviz_cmd = Node(
    condition=IfCondition(use_rviz),
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen')    


  # Create the launch description and populate
  return LaunchDescription([
    # Declare the launch options
    declare_use_rviz_cmd,

    # Add any actions
    # start_robot_localization_global_cmd,
    start_robot_localization_local_cmd,
    start_rviz_cmd,
    start_gps_point_to_pose_cmd
  ])

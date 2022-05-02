from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/Summit_XL_Steel/Velodyne_VLP_16/point_cloud'),
                        ('scan', '/scan')],
            parameters=[{
                'target_frame': 'Velodyne_VLP_16',
                'transform_tolerance': 0.01,
                'min_height': -0.52,
                'max_height': 0.5,
                'angle_min': -3.1415926,  # -M_PI/2
                'angle_max': 3.1415926,  # M_PI/2
                'angle_increment': 0.00174533,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.3,
                'range_max': 50.0,
                'use_inf': False,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        )
    ])

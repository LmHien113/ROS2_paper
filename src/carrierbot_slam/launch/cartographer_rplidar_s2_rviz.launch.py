#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Get the package directories
    carrierbot_slam_prefix = get_package_share_directory('carrierbot_slam')
    rplidar_ros_prefix = get_package_share_directory('rplidar_ros')

    # RViz config file
    rviz_config_file = os.path.join(carrierbot_slam_prefix, 'rviz', 'cartographer.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # Include the cartographer launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(rplidar_ros_prefix, 'launch'),
                '/cartographer_rplidar_s2.launch.py'
            ]),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        # RViz2 node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
    ])
#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Get the package directories
    rplidar_ros_prefix = get_package_share_directory('rplidar_ros')
    carrierbot_slam_prefix = get_package_share_directory('carrierbot_slam')

    # RViz config file
    rviz_config_file = os.path.join(carrierbot_slam_prefix, 'rviz', 'cartographer.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # Include the cartographer launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(rplidar_ros_prefix, 'launch'),
                '/cartographer_rplidar_s2.launch.py'
            ]),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        # RViz2 node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
    ])

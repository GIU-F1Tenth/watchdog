#!/usr/bin/env python3

"""
F1TENTH Watchdog Launch File

Launches the watchdog system integrated with F1TENTH vehicle components.
Includes proper topic remapping and parameter configuration for F1TENTH standard topics.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for F1TENTH integrated watchdog."""
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('watchdog'),
            'config',
            'f1tenth_params.yaml'
        ]),
        description='Path to the watchdog configuration file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace'
    )
    
    # Watchdog node with F1TENTH topic remapping
    watchdog_node = Node(
        package='watchdog',
        executable='watchdog_node.py',
        name='watchdog',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            # F1TENTH standard topic mappings
            ('/sensors/core', '/sensors/core'),  # VESC telemetry
            ('/scan', '/scan'),  # LiDAR
            ('/camera/image_raw', '/camera/camera/color/image_raw'),  # Camera
            ('/odom', '/odom'),  # Odometry
            ('/imu', '/imu'),  # IMU data
            
            # Watchdog output topics
            ('/watchdog/sanity_warnings', '/watchdog/sanity_warnings'),
            ('/watchdog/sensor_health', '/watchdog/sensor_health'),
            ('/watchdog/sanity_summary', '/watchdog/sanity_summary'),
            ('/tmp/watchdog/critical', '/emergency_stop'),  # Map to F1TENTH emergency stop
        ],
        output='screen',
        emulate_tty=True,
    )
    
    # Log information
    log_info = LogInfo(
        msg="Starting F1TENTH Watchdog System with sanity checking enabled"
    )
    
    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        namespace_arg,
        log_info,
        watchdog_node,
    ])
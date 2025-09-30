#!/usr/bin/env python3

"""
Complete F1TENTH System Launch with Watchdog Integration

This launch file demonstrates how to integrate the watchdog system
with a complete F1TENTH autonomous racing stack.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate complete F1TENTH system launch description."""
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    enable_watchdog_arg = DeclareLaunchArgument(
        'enable_watchdog',
        default_value='true',
        description='Enable watchdog monitoring'
    )
    
    enable_sanity_checking_arg = DeclareLaunchArgument(
        'enable_sanity_checking',
        default_value='true',
        description='Enable sanity checking features'
    )
    
    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='',
        description='Robot namespace'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'watchdog_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('watchdog'),
            'config',
            'f1tenth_params.yaml'
        ]),
        description='Watchdog configuration file'
    )
    
    # Global parameters
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_namespace = LaunchConfiguration('robot_namespace')
    
    # Set global use_sim_time parameter
    set_use_sim_time = SetParameter('use_sim_time', use_sim_time)
    
    # Watchdog system nodes
    watchdog_group = GroupAction([
        
        # Main watchdog node
        Node(
            package='watchdog',
            executable='watchdog_node.py',
            name='watchdog',
            namespace=robot_namespace,
            parameters=[
                LaunchConfiguration('watchdog_config'),
                {'use_sim_time': use_sim_time}
            ],
            remappings=[
                # F1TENTH standard topics
                ('/sensors/core', '/sensors/core'),
                ('/scan', '/scan'),
                ('/camera/image_raw', '/camera/camera/color/image_raw'),
                ('/odom', '/odom'),
                ('/imu', '/imu'),
                
                # Emergency stop integration
                ('/tmp/watchdog/critical', '/emergency_stop'),
                
                # Watchdog-specific topics
                ('/watchdog/sanity_warnings', '/watchdog/sanity_warnings'),
                ('/watchdog/sensor_health', '/watchdog/sensor_health'),
                ('/watchdog/sanity_summary', '/watchdog/sanity_summary'),
            ],
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_watchdog'))
        ),
        
        # FSM Integration node (commented out - executable not found)
        # Node(
        #     package='watchdog',
        #     executable='fsm_integration_node',
        #     name='watchdog_fsm_integration',
        #     namespace=robot_namespace,
        #     parameters=[
        #         {'use_sim_time': use_sim_time}
        #     ],
        #     remappings=[
        #         # Connect to watchdog outputs
        #         ('/watchdog/sanity_summary', '/watchdog/sanity_summary'),
        #         ('/watchdog/sanity_warnings', '/watchdog/sanity_warnings'),
        #         ('/watchdog/sensor_health', '/watchdog/sensor_health'),
        #         
        #         # FSM communication
        #         ('/fsm/state', '/fsm/state'),
        #         ('/emergency_stop', '/emergency_stop'),
        #         ('/fsm/watchdog_health', '/fsm/watchdog_health'),
        #         ('/racing/system_status', '/racing/system_status'),
        #         
        #         # Emergency control override
        #         ('/cmd_vel_emergency', '/cmd_vel_emergency'),
        #     ],
        #     output='screen',
        #     condition=IfCondition(LaunchConfiguration('enable_watchdog'))
        # ),
        
    ], condition=IfCondition(LaunchConfiguration('enable_watchdog')))
    
    # Example F1TENTH system nodes (commented out - replace with actual nodes)
    """
    f1tenth_system_group = GroupAction([
        
        # VESC Driver
        Node(
            package='vesc_driver',
            executable='vesc_driver_node',
            name='vesc_driver',
            namespace=robot_namespace,
            parameters=[vesc_config],
            remappings=[
                ('sensors/core', '/sensors/core'),
                ('sensors/servo_position_command', '/vesc/servo_position_command'),
                ('sensors/ackermann_cmd', '/vesc/ackermann_cmd'),
            ]
        ),
        
        # LiDAR Driver
        Node(
            package='urg_node',
            executable='urg_node',
            name='urg_node',
            namespace=robot_namespace,
            parameters=[lidar_config],
            remappings=[
                ('scan', '/scan'),
            ]
        ),
        
        # Camera Driver
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense',
            namespace=robot_namespace,
            parameters=[camera_config],
            remappings=[
                ('color/image_raw', '/camera/camera/color/image_raw'),
                ('depth/image_rect_raw', '/camera/depth/image_rect_raw'),
            ]
        ),
        
        # Localization
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            namespace=robot_namespace,
            parameters=[ekf_config],
            remappings=[
                ('odometry/filtered', '/odom'),
            ]
        ),
        
        # FSM Node
        Node(
            package='f1tenth_stack',
            executable='fsm_node',
            name='fsm',
            namespace=robot_namespace,
            parameters=[fsm_config],
            remappings=[
                ('emergency_stop', '/emergency_stop'),
                ('fsm/state', '/fsm/state'),
            ]
        ),
        
    ])
    """
    
    # Log messages
    log_start = LogInfo(
        msg="Starting F1TENTH system with integrated watchdog monitoring"
    )
    
    log_watchdog = LogInfo(
        msg="Watchdog system enabled with sanity checking",
        condition=IfCondition(LaunchConfiguration('enable_watchdog'))
    )
    
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        enable_watchdog_arg,
        enable_sanity_checking_arg,
        robot_namespace_arg,
        config_file_arg,
        
        # Global parameters
        set_use_sim_time,
        
        # Log messages
        log_start,
        log_watchdog,
        
        # Node groups
        watchdog_group,
        # f1tenth_system_group,  # Uncomment and configure for full system
    ])
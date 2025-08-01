#!/usr/bin/env python3
"""
Topic-based Vehicle Interface Launch File

This launch file demonstrates how to use topic-based descriptions
instead of file-based descriptions for Autoware integration.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description"""
    
    # Declare arguments
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'vehicle_role_name',
            default_value='ego_vehicle',
            description='Role name of the vehicle in CARLA'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'carla_host',
            default_value='localhost',
            description='CARLA server host'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'carla_port',
            default_value='2000',
            description='CARLA server port'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        )
    )
    
    # Create nodes
    
    # Description publisher - publishes descriptions to topics
    description_publisher = Node(
        package='carla_sensor_kit_utils',
        executable='description_publisher.py',
        name='description_publisher',
        output='screen',
        parameters=[{
            'carla_host': LaunchConfiguration('carla_host'),
            'carla_port': LaunchConfiguration('carla_port'),
            'vehicle_role_name': LaunchConfiguration('vehicle_role_name'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # Robot state publisher - reads from topic
    # Note: We start it without robot_description parameter
    # It will get the description from the topic published by description_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            # No robot_description parameter - will get from topic
        }],
        remappings=[
            # Remap to use our custom topic if needed
            ('/robot_description', '/robot_description')
        ]
    )
    
    # Vehicle interface node
    vehicle_interface = Node(
        package='carla_vehicle_launch',
        executable='carla_vehicle_interface.py',
        name='carla_vehicle_interface',
        output='screen',
        parameters=[{
            'carla_host': LaunchConfiguration('carla_host'),
            'carla_port': LaunchConfiguration('carla_port'),
            'vehicle_role_name': LaunchConfiguration('vehicle_role_name'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # For Autoware integration, we need to publish calibration data
    # This would normally come from YAML files, but we're publishing as topics
    calibration_publisher = Node(
        package='carla_sensor_kit_utils',
        executable='calibration_publisher.py',
        name='calibration_publisher',
        output='screen',
        parameters=[{
            'vehicle_role_name': LaunchConfiguration('vehicle_role_name'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # Create launch description
    ld = LaunchDescription(declared_arguments)
    
    # Add nodes
    ld.add_action(description_publisher)
    ld.add_action(robot_state_publisher)
    ld.add_action(vehicle_interface)
    
    # Log info about topic-based approach
    ld.add_action(LogInfo(
        msg="Using topic-based robot description approach. "
            "Robot description will be published to /robot_description topic."
    ))
    
    return ld
"""
Launch file for walker robot simulation with Webots.

Copyright (c) 2024 Tirth Sadaria
Licensed under the Apache License, Version 2.0
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
# Using IncludeLaunchDescription to use the original TurtleBot launch file
# which handles all controller setup correctly


def generate_launch_description():
    # Get package directory
    package_dir = FindPackageShare(package='walker')
    package_share_dir = get_package_share_directory('walker')
    turtlebot_package_dir = get_package_share_directory('webots_ros2_turtlebot')
    
    # Get source directory for bag file storage (not install directory)
    # Walk up from share/walker to find the workspace root
    source_results_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(package_share_dir))),
        'src', 'walker', 'results'
    )
    
    # Launch arguments
    record_bag_arg = DeclareLaunchArgument(
        'record_bag',
        default_value='false',
        description='Flag to enable rosbag recording'
    )
    
    # Include the original TurtleBot launch file - this handles all the controller setup correctly
    # The world file is in the turtlebot package, so we use the default
    turtlebot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot_package_dir, 'launch', 'robot_launch.py')
        ),
        launch_arguments={
            'world': 'turtlebot3_burger_example.wbt',  # This is in turtlebot package
            'mode': 'realtime'
        }.items()
    )
    
    # Walker node - publish to /cmd_vel
    # The turtlebot launch file should handle remapping to the controller
    # If that doesn't work, we'll try direct remapping to /diffdrive_controller/cmd_vel_unstamped
    walker_node = Node(
        package='walker',
        executable='walker_node',
        name='walker_node',
        output='screen',
        parameters=[{
            'obstacle_threshold': 0.5,
            'forward_velocity': 0.2,
            'angular_velocity': 0.5,
            'timer_period': 0.1
        }]
        # No remapping - let turtlebot launch handle it, or publish directly to /cmd_vel
    )
    
    # Delay walker node to ensure controllers are active
    delayed_walker = TimerAction(
        period=5.0,  # Wait 5 seconds for controllers to activate
        actions=[walker_node]
    )
    
    # Rosbag recording node (conditional)
    record_bag = LaunchConfiguration('record_bag')
    # Ensure results directory exists in source directory
    os.makedirs(source_results_dir, exist_ok=True)
    # Create timestamped bag name
    from datetime import datetime
    bag_name = f"walker_bag_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    
    # Build rosbag command
    rosbag_cmd = [
        'ros2', 'bag', 'record',
        '-a',  # Record all topics
        '-x', '/camera/.*',  # Exclude camera topics
        '--output', os.path.join(source_results_dir, bag_name)
    ]
    
    rosbag_node = ExecuteProcess(
        condition=IfCondition(record_bag),
        cmd=rosbag_cmd,
        output='screen'
    )
    
    return LaunchDescription([
        record_bag_arg,
        turtlebot_launch,  # This includes Webots, controllers, etc.
        delayed_walker,  # Start walker node after controllers are active
        rosbag_node,
    ])


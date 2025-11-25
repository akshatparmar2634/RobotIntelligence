#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'auto_start',
            default_value='true',
            description='Whether to automatically start exploration'
        ),
        
        DeclareLaunchArgument(
            'use_vlm_node',
            default_value='true',
            description='Use VLM node for detailed object descriptions'
        ),
        
        # Autonomous Navigation Node
        Node(
            package='ri_pkg',
            executable='autonomous_navigation_node',
            name='autonomous_navigation_node',
            output='screen',
            parameters=[
                {'auto_start': LaunchConfiguration('auto_start')}
            ]
        ),
        
        # # YOLO VLM Node for semantic mapping (primary)
        # Note: VLM node should be run separately in terminal with virtual environment
        # Run: source ~/yolo_clip_env/bin/activate && ros2 run ri_pkg yolo_improved_vlm_node
        # Node(
        #     package='ri_pkg',
        #     executable='yolo_improved_vlm_node',
        #     name='yolo_improved_vlm_node',
        #     output='screen',
        #     condition=IfCondition(LaunchConfiguration('use_vlm_node'))
        # ),
        
        # # YOLO CLIP Node for semantic mapping (alternative)
        # Note: CLIP node should be run separately in terminal with virtual environment  
        # Run: source ~/yolo_clip_env/bin/activate && ros2 run ri_pkg yolo_clip_camera_node
        # Node(
        #     package='ri_pkg',
        #     executable='yolo_clip_camera_node',
        #     name='yolo_clip_camera_node',
        #     output='screen',
        #     condition=IfCondition(LaunchConfiguration('use_vlm_node'))
        # ),
        
        # Camera subscriber for visualization
        Node(
            package='ri_pkg',
            executable='camera_subscriber',
            name='camera_subscriber',
            output='screen'
        ),
        
        LogInfo(
            msg=[
                "Autonomous Navigation System Started!\n",
                "- Navigation node: Handles waypoint navigation\n",
                "- Camera subscriber: Shows camera feed\n",
                "- IMPORTANT: Run semantic mapping node separately:\n",
                "  Terminal 4: source ~/yolo_clip_env/bin/activate && ros2 run ri_pkg yolo_improved_vlm_node\n",
                "Use 'ros2 run ri_pkg navigation_controller' for manual control"
            ]
        )
    ])
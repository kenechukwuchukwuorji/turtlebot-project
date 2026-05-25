#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_odom = LaunchConfiguration('launch_odom')
    launch_nav = LaunchConfiguration('launch_nav')
    launch_obstacle_detector = LaunchConfiguration('launch_obstacle_detector')
    launch_caps = LaunchConfiguration('launch_caps')
    caps_visual_servo = LaunchConfiguration('caps_visual_servo')

    odom_topic = LaunchConfiguration('odom_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    nav_cmd_topic = LaunchConfiguration('nav_cmd_topic')
    final_cmd_topic = LaunchConfiguration('final_cmd_topic')
    image_topic = LaunchConfiguration('image_topic')

    return LaunchDescription([
        DeclareLaunchArgument('launch_odom', default_value='true'),
        DeclareLaunchArgument('launch_nav', default_value='true'),
        DeclareLaunchArgument('launch_obstacle_detector', default_value='false'),
        DeclareLaunchArgument('launch_caps', default_value='false'),
        DeclareLaunchArgument('caps_visual_servo', default_value='true'),
        DeclareLaunchArgument('odom_topic', default_value='/turtlebot_odom'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('nav_cmd_topic', default_value='/cmd_vel_nav'),
        DeclareLaunchArgument('final_cmd_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),

        Node(
            package='turtlebot_odom_pkg',
            executable='odom',
            name='turtlebot_odom',
            output='screen',
            condition=IfCondition(launch_odom),
        ),

        Node(
            package='turtlenav_traj_pkg',
            executable='turtle_nav',
            name='turtle_nav_node',
            output='screen',
            condition=IfCondition(launch_nav),
            parameters=[{
                'odom_topic': odom_topic,
                'scan_topic': scan_topic,
                'cmd_vel_topic': nav_cmd_topic,
            }],
        ),

        Node(
            package='obstacles',
            executable='obstacle_detector',
            name='obstacle_detector',
            output='screen',
            condition=IfCondition(launch_obstacle_detector),
        ),

        Node(
            package='cap_detector_pkg',
            executable='task4_caps',
            name='task4_camera_caps',
            output='screen',
            condition=IfCondition(launch_caps),
            parameters=[{
                'image_topic': image_topic,
                'input_cmd_vel_topic': nav_cmd_topic,
                'output_cmd_vel_topic': final_cmd_topic,
                'use_stamped_cmd_vel': True,
                'enable_visual_servo': caps_visual_servo,
            }],
        ),
    ])

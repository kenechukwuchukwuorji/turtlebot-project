import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    # -------------------------------------------------
    # Arguments
    # -------------------------------------------------

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )

    stamped_cmd_arg = DeclareLaunchArgument(
        'use_stamped_cmd_vel',
        default_value='false',
        description='Use stamped cmd_vel messages'
    )

    visual_servo_arg = DeclareLaunchArgument(
        'enable_visual_servo',
        default_value='true',
        description='Enable visual servoing'
    )

    stop_area_arg = DeclareLaunchArgument(
        'stop_area',
        default_value='1000000.0',
        description='Contour area threshold to stop robot'
    )

    # -------------------------------------------------
    # Paths
    # -------------------------------------------------

    pkg_share = get_package_share_directory('turtlenav_traj_pkg')

    rviz_config_path = os.path.join(
        pkg_share,
        'config',
        'turtlebot3_config.rviz'
    )

    # -------------------------------------------------
    # Nodes
    # -------------------------------------------------

    turtle_nav_node = Node(
        package='turtlenav_traj_pkg',
        executable='turtle_nav',
        name='turtle_nav_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_stamped_cmd_vel': LaunchConfiguration('use_stamped_cmd_vel')
        }],
        remappings=[
            ('/cmd_vel', '/cmd_vel_nav')
        ]
    )

    cap_detector_node = Node(
        package='cap_detector_pkg',
        executable='task4_caps',
        name='task4_caps',
        output='screen',
        parameters=[{
            'use_stamped_cmd_vel': LaunchConfiguration('use_stamped_cmd_vel'),
            'image_topic': '/camera/image_raw',
            'input_cmd_vel_topic': '/cmd_vel_nav',
            'output_cmd_vel_topic': '/cmd_vel',
            'enable_visual_servo': LaunchConfiguration('enable_visual_servo'),
            'min_area': 100.0,
            'stop_area': LaunchConfiguration('stop_area')
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([

        sim_time_arg,
        stamped_cmd_arg,
        visual_servo_arg,
        stop_area_arg,

        turtle_nav_node,
        cap_detector_node,
        rviz_node
    ])
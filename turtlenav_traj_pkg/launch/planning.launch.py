import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Locate our package directories
    pkg_share = get_package_share_directory('turtlenav_traj_pkg')
    

    rviz_config_path = os.path.join(pkg_share, 'config', 'turtlebot3_config.rviz')

    # 3. Define the main navigation node
    turtle_nav_node = Node(
        package='turtlenav_traj_pkg',
        executable='turtle_nav',
        name='turtle_nav_node',
        output='screen',
        parameters=[{
            'use_sim_time': True, 
        }]
    )

    # 4. Define the RViz2 node loading your specific configuration file
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # 5. Build the launch description to execute both nodes simultaneously
    return LaunchDescription([
        turtle_nav_node,
        rviz_node
    ])
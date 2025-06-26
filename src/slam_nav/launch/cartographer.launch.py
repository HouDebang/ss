from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    slam_nav_dir = get_package_share_directory('slam_nav')
    config_dir = os.path.join(slam_nav_dir, 'config')
    
    return LaunchDescription([
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            arguments=['-configuration_directory', config_dir, '-configuration_basename', 'robot.lua'],
            parameters=[{'use_sim_time': True}]  # 啟用仿真時間
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'],
            parameters=[{'use_sim_time': True}]  # 啟用仿真時間
        )
    ])
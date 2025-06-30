from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    pkg_share = os.path.join(os.path.dirname(__file__), '..')
    urdf_path = os.path.join(pkg_share, 'urdf', 'ugv_rover.urdf')
    with open(urdf_path, 'r') as inf:
        robot_description = inf.read()

    return LaunchDescription([

        # 启动小车底盘节点
        Node(
            package='slam_nav',
            executable='chassis_cmdvel_node',
            name='chassis_cmdvel_node',
            output='screen',
            parameters=[{'port': '/dev/car_controller', 'baudrate': 115200}]
        ),
        # 启动robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        # 启动joint_state_publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'rate': 20}], 
            output='screen'
        ),
    ])

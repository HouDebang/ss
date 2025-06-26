import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('slam_nav')
    urdf_file = os.path.join(pkg_share, 'urdf', 'ugv_rover.urdf')
    world_file = os.path.join(pkg_share, 'worlds', 'world0.world')
    config_file = os.path.join(pkg_share, 'config', 'bridge.yaml')  # YAML 文件路径

    # 读取 URDF 文件内容
    with open(urdf_file, 'r') as urdf:
        robot_description = urdf.read()

    use_gazebo_gui = DeclareLaunchArgument(
        'use_gazebo_gui',
        default_value='true',
        description='Whether to launch Gazebo GUI'
    )

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': [world_file if LaunchConfiguration('use_gazebo_gui').__eq__(TextSubstitution(text='true')) else f'-s {world_file}']
        }.items()
    )

    spawn_model_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'ros_gz_spawn_model.launch.py')
        ]),
        launch_arguments={
            'world': 'demo',#world名称
            'file': urdf_file,
            'entity_name': 'ugv_rover',
            'bridge_name': 'ugv_rover_bridge',
            'config_file': config_file,  # 引用 YAML 文件
            'x': '0.0',
            'y': '0.0',
            'z': '0.2'
        }.items()
    )

    delayed_spawn = TimerAction(period=10.0, actions=[spawn_model_launch])

    return LaunchDescription([
        use_gazebo_gui,
        gz_sim_launch,
        delayed_spawn,
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge',
            output='screen',
            parameters=[{'config_file': config_file}, {'use_sim_time': True}],
        ),

        # 使用 parameters 参数传递 robot_description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        
    ])
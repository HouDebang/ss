import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('slam_nav')
    world_file = os.path.join(pkg_share, 'worlds', 'school.world')

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

    return LaunchDescription([
        use_gazebo_gui,
        gz_sim_launch,
    ])
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # 激光传感器发布节点
        Node(
            package='demo_python_service',  # 替换为你的包名
            executable='lasersensor_publisher_node',
            name='lasersensor_publisher',
            output='screen',
            parameters=[{
                'device_port': '/dev/wheeltec_controller',  # 串口设备路径
                'baud_rate': 230400,  # 波特率
                'timeout': 0.1  # 超时时间
            }]
        ),
        
        # 人脸检测服务节点
        Node(
            package='demo_python_service',  # 替换为你的包名
            executable='face_detect_node',
            name='face_detect_node',
            output='screen',
            parameters=[{
                'model': 'hog',  # 使用HOG模型（CPU）
                'number_of_times_to_upsample': 1,  # 上采样次数
                'default_image_path': PathJoinSubstitution([
                    FindPackageShare('demo_python_service'),  # 替换为包含默认图片的包名
                    'resource',
                    'default.jpg'
                ])
            }]
        ),
        
        # 摄像头节点（改造后的版本）
        Node(
            package='demo_python_service',  # 替换为你的包名
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[{
                'camera_index': 0,  # 摄像头索引
                'frame_rate': 10.0,  # 帧率 (Hz)
                'display_window': True  # 是否显示视频窗口
            }]
        )
    ])
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 声明云台控制参数
    gimbal_serial_port_arg = DeclareLaunchArgument(
        'gimbal_serial_port',
        default_value='/dev/ttyUSB0',
        description='云台串口设备路径'
    )
    
    gimbal_baud_rate_arg = DeclareLaunchArgument(
        'gimbal_baud_rate',
        default_value='115200',
        description='云台串口波特率'
    )
    
    gimbal_dead_zone_arg = DeclareLaunchArgument(
        'gimbal_dead_zone',
        default_value='30',
        description='云台控制死区像素大小'
    )
    
    gimbal_kp_arg = DeclareLaunchArgument(
        'gimbal_kp',
        default_value='0.15',
        description='云台控制比例系数'
    )
    
    return LaunchDescription([
        # 声明参数
        gimbal_serial_port_arg,
        gimbal_baud_rate_arg,
        gimbal_dead_zone_arg,
        gimbal_kp_arg,
        
        # 激光传感器发布节点
        Node(
            package='demo_python_service',
            executable='lasersensor_publisher_node',
            name='lasersensor_publisher',
            output='screen',
            parameters=[{
                'device_port': '/dev/wheeltec_controller',
                'baud_rate': 230400,
                'timeout': 0.1
            }]
        ),
        
        # 人脸检测服务节点
        Node(
            package='demo_python_service',
            executable='face_detect_node',
            name='face_detect_node',
            output='screen',
            parameters=[{
                'model': 'hog',
                'number_of_times_to_upsample': 1,
                'default_image_path': PathJoinSubstitution([
                    FindPackageShare('demo_python_service'),
                    'resource',
                    'default.jpg'
                ])
            }]
        ),
        
        # 摄像头节点
        Node(
            package='demo_python_service',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[{
                'camera_index': 0,
                'frame_rate': 10.0,
                'display_window': True
            }]
        ),
        
        # 新增云台控制节点
        Node(
            package='demo_python_service',
            executable='gimbal_node',  # 假设可执行文件名为gimbal_node
            name='gimbal_control_node',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('gimbal_serial_port'),
                'baud_rate': LaunchConfiguration('gimbal_baud_rate'),
                'dead_zone': LaunchConfiguration('gimbal_dead_zone'),
                'kp': LaunchConfiguration('gimbal_kp'),
                # 添加更多参数
                'image_width': 640,  # 默认图像宽度
                'image_height': 480,  # 默认图像高度
                'max_angle': 30.0,   # 最大角度限制
            }],
            arguments=['--ros-args', '--log-level', 'info']  # 设置日志级别
        )
    ])
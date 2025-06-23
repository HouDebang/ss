import rclpy
from rclpy.node import Node
from chapt4_interfaces.msg import FacePosition
import serial
import json
import math

class GimbalControlNode(Node):
    def __init__(self):
        super().__init__('gimbal_control_node')
        self.subscription = self.create_subscription(
            FacePosition, 
            'face_positions',  # Fixed topic name
            self.face_position_callback, 
            10
        )
        self.get_logger().info("云台控制节点已启动")
        # 云台控制参数
        self.dead_zone = 20  # 中心死区像素大小
        self.kp = 0.1  # 比例控制系数
         # ... 新增PID状态变量
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.control_period = 0.1  # 与摄像头频率匹配
        # 初始化云台串口连接
        try:
            self.ser = serial.Serial('/dev/base_controller', 115200, timeout=1)
            self.get_logger().info("云台串口连接成功")
            # 初始化云台位置
            self.move_gimbal(0, 0)
        except Exception as e:
            self.get_logger().error(f"云台串口连接失败: {str(e)}")
            self.ser = None
    
    def face_position_callback(self, msg):
        if msg.number == 0:
            return  # 未检测到人脸
        image_width = msg.image_width
        image_height = msg.image_height
        self.get_logger().info(f"图像尺寸: {image_width}x{image_height}")
        # 只处理检测到的第一个人脸
        top = msg.top[0]
        right = msg.right[0]
        bottom = msg.bottom[0]
        left = msg.left[0]
        
        # 计算人脸中心点
        face_center_x = (left + right) / 2
        face_center_y = (top + bottom) / 2
        
        # 计算与图像中心的偏移
        offset_x = face_center_x - (image_width / 2)
        offset_y = face_center_y - (image_height / 2)
        self.integral_x += offset_x * self.control_period
        self.integral_y += offset_y * self.control_period
        derivative_x = (offset_x - self.prev_error_x) / self.control_period
        derivative_y = (offset_y - self.prev_error_y) / self.control_period
        
        angle_x = -(self.kp * offset_x + self.ki * self.integral_x + self.kd * derivative_x)
        angle_y = -(self.kp * offset_y + self.ki * self.integral_y + self.kd * derivative_y)
        self.prev_error_x, self.prev_error_y = offset_x, offset_y
        # 检查是否在死区内
        if abs(offset_x) < self.dead_zone and abs(offset_y) < self.dead_zone:
            return  # 人脸已在中心区域，无需调整
        self.get_logger().info(f"人脸位于死区内 (X:{offset_x:.1f}, Y:{offset_y:.1f})，不调整")
        # 计算云台需要调整的角度
        # 角度与偏移量成比例关系
        angle_x = offset_x * self.kp  # 水平方向调整
        angle_y = offset_y * self.kp  # 垂直方向调整
        
        # 限制角度范围
        angle_x = max(min(angle_x, 30), -30)  # ±30度限制
        angle_y = max(min(angle_y, 30), -30)  # ±30度限制
        
        self.get_logger().info(
            f"人脸中心: ({face_center_x:.1f}, {face_center_y:.1f}) | "
            f"图像中心: ({image_width/2:.1f}, {image_height/2:.1f}) | "
            f"偏移量: X={offset_x:.1f}, Y={offset_y:.1f}"
        )
        
        # 发送云台控制指令
        self.move_gimbal(angle_x, angle_y)
    
    def move_gimbal(self, angle_x, angle_y=0, speed=30, acc=10):
        """发送云台控制指令"""
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warning("云台串口未连接")
            return
        
        cmd = {
            "T": 133, 
            "X": angle_x, 
            "Y": angle_y, 
            "SPD": speed, 
            "ACC": acc
        }
        command_str = json.dumps(cmd) + "\n"
        try:
            self.ser.write(command_str.encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f"云台控制失败: {str(e)}")
    
    def __del__(self):
        if hasattr(self, 'ser') and self.ser:
            self.ser.close()

def main():
    rclpy.init()
    node = GimbalControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到键盘中断，准备退出...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
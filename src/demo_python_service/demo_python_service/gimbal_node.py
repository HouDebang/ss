import rclpy
from rclpy.node import Node
from chapt4_interfaces.msg import FacePosition
import serial
import json
import math

class GimbalControlNode(Node):
    def __init__(self):
        super().__init__('gimbal_control_node')
        # 订阅人脸位置信息
        self.subscription = self.create_subscription(
            FacePosition, 
            'face_position', 
            self.face_position_callback, 
            10
        )
        self.get_logger().info("云台控制节点已启动")
        
        # 云台控制参数
        self.image_width = 640  # 假设图像宽度
        self.image_height = 480  # 假设图像高度
        self.dead_zone = 20  # 中心死区像素大小
        self.kp = 0.1  # 比例控制系数
        
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
        
        # 只处理检测到的第一个人脸
        top = msg.top[0]
        right = msg.right[0]
        bottom = msg.bottom[0]
        left = msg.left[0]
        
        # 计算人脸中心点
        face_center_x = (left + right) / 2
        face_center_y = (top + bottom) / 2
        
        # 计算与图像中心的偏移
        offset_x = face_center_x - (self.image_width / 2)
        offset_y = face_center_y - (self.image_height / 2)
        
        # 检查是否在死区内
        if abs(offset_x) < self.dead_zone and abs(offset_y) < self.dead_zone:
            return  # 人脸已在中心区域，无需调整
        
        # 计算云台需要调整的角度
        # 角度与偏移量成比例关系
        angle_x = -offset_x * self.kp  # 水平方向调整
        angle_y = -offset_y * self.kp  # 垂直方向调整
        
        # 限制角度范围
        angle_x = max(min(angle_x, 30), -30)  # ±30度限制
        angle_y = max(min(angle_y, 30), -30)  # ±30度限制
        
        self.get_logger().info(
            f"调整云台: X={angle_x:.2f}°, Y={angle_y:.2f}° "
            f"(偏移量: X={offset_x:.1f}px, Y={offset_y:.1f}px)"
        )
        
        # 发送云台控制指令
        self.move_gimbal(angle_x, angle_y)
    
    def move_gimbal(self, angle_x, angle_y=0, speed=30, acc=10):
        """发送云台控制指令"""
        if self.ser is None:
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
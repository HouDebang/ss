#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import threading
import json

class SerialController(Node):
    def __init__(self):
        super().__init__('serial_controller')
        # 声明参数
        self.declare_parameter('port', '/dev/car_controller')
        self.declare_parameter('baudrate', 115200)
        
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        
        # 初始化串口
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                dsrdtr=None
            )
            self.ser.setRTS(False)
            self.ser.setDTR(False)
            self.get_logger().info(f"成功连接到串口: {port}")
        except serial.SerialException as e:
            self.get_logger().error(f"无法打开串口: {e}")
            rclpy.shutdown()
            return
        
        # 启动串口接收线程
        self.serial_recv_thread = threading.Thread(target=self.read_serial)
        self.serial_recv_thread.daemon = True
        self.serial_recv_thread.start()
        
        # 订阅速度指令
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.subscription  # 防止未使用变量警告
    
    def cmd_vel_callback(self, msg):
        """处理速度指令消息"""
        # 构造JSON命令
        command = {
            'linear': msg.linear.x,
            'angular': msg.angular.z
        }
        json_command = json.dumps(command) + '\n'
        
        # 发送到串口
        try:
            self.ser.write(json_command.encode())
            self.get_logger().debug(f"Sent: {json_command.strip()}")
        except Exception as e:
            self.get_logger().error(f"串口写入错误: {e}")
    
    def read_serial(self):
        """持续读取串口数据的线程函数"""
        while rclpy.ok():
            try:
                if self.ser.in_waiting:
                    data = self.ser.readline().decode('latin-1').strip()
                    self.get_logger().info(f"Received: {data}")
            except Exception as e:
                self.get_logger().error(f"串口读取错误: {e}")
                break
    
    def cleanup(self):
        """关闭串口"""
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("串口已关闭")

def main(args=None):
    rclpy.init(args=args)
    node = SerialController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
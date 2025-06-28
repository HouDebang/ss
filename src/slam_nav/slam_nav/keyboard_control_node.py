#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

class KeyboardListener(Node):
    def __init__(self):
        super().__init__('keyboard_listener')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 速度参数
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.velocity_step = 0.1
        self.max_linear_velocity = 1.0
        
        # 保存终端设置
        self.old_attr = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info("键盘控制已启动 (WASD控制方向, 空格停止, Q退出)")
        self.get_logger().info("速度参数: 线性: {:.1f} m/s, 角速度: {:.1f} rad/s".format(
            self.linear_velocity, self.angular_velocity
        ))
        
        # 定时器用于发布速度
        self.timer = self.create_timer(0.1, self.publish_velocity)
    
    def get_key(self):
        """获取键盘输入"""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_attr)
        return key
    
    def publish_velocity(self):
        """处理键盘输入并发布速度指令"""
        key = self.get_key()
        
        if key == 'q' or key == 'Q':
            self.cleanup()
            rclpy.shutdown()
            return
        
        # 处理运动控制命令
        if key == 'w':
            self.linear_velocity = min(self.linear_velocity + self.velocity_step, 
                                      self.max_linear_velocity)
        elif key == 's':
            self.linear_velocity = max(self.linear_velocity - self.velocity_step, 
                                      -self.max_linear_velocity)
        elif key == 'a':
            self.angular_velocity = min(self.angular_velocity + self.velocity_step, 
                                      self.max_linear_velocity)
        elif key == 'd':
            self.angular_velocity = max(self.angular_velocity - self.velocity_step, 
                                      -self.max_linear_velocity)
        elif key == ' ':
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0
        
        # 显示当前速度
        if key in ['w', 's', 'a', 'd', ' ']:
            self.get_logger().info("速度参数: 线性: {:.1f} m/s, 角速度: {:.1f} rad/s".format(
                self.linear_velocity, self.angular_velocity
            ))
        
        # 发布速度指令
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_velocity
        twist_msg.angular.z = self.angular_velocity
        self.publisher.publish(twist_msg)
    
    def cleanup(self):
        """清理资源"""
        self.get_logger().info("正在关闭键盘监听节点...")
        # 发布停止指令
        stop_msg = Twist()
        self.publisher.publish(stop_msg)
        # 恢复终端设置
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_attr)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardListener()
    
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
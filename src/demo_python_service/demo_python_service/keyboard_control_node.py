import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import os
import sys
sys.path.append(os.path.expanduser("~/face_recognition_evn/lib/python3.12/site-packages"))
from pynput import keyboard

# 按键与速度映射
KEY_BINDINGS = {
    'w': (0.2, 0.0),   # 前进
    's': (-0.2, 0.0),  # 后退
    'a': (0.0, 0.5),   # 左转
    'd': (0.0, -0.5),  # 右转
    'q': (0.0, 0.0),   # 停止
}

class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_control_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        print("按WASD控制小车，Q停止，ESC退出。")

    def on_press(self, key):
        try:
            k = key.char.lower()
        except AttributeError:
            return
        if k in KEY_BINDINGS:
            linear, angular = KEY_BINDINGS[k]
            twist = Twist()
            twist.linear.x = linear
            twist.angular.z = angular
            self.publisher_.publish(twist)
            print(f"按下{k.upper()}，线速度: {linear}, 角速度: {angular}")
        if k == '\x1b':  # ESC退出
            print("退出键盘控制。"); return False

    def run(self):
        with keyboard.Listener(on_press=self.on_press) as listener:
            listener.join()

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    try:
        node.run()
    except KeyboardInterrupt:
        print("退出键盘控制。")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main() 
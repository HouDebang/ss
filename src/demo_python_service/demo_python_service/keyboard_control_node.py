import time
from pynput import keyboard
from chassis_control_client import ChassisSerialClient

# 按键与速度映射
KEY_BINDINGS = {
    'w': (0.2, 0.2),   # 前进
    's': (-0.2, -0.2), # 后退
    'a': (-0.1, 0.1),  # 左转
    'd': (0.1, -0.1),  # 右转
    'q': (0.0, 0.0),   # 停止
}

class KeyboardControl:
    def __init__(self, port="/dev/car_controller"):
        self.client = ChassisSerialClient(port)
        self.last_cmd = (0.0, 0.0)
        print("按WASD控制小车，Q停止，ESC退出。")

    def on_press(self, key):
        try:
            k = key.char.lower()
        except AttributeError:
            return
        if k in KEY_BINDINGS:
            speeds = KEY_BINDINGS[k]
            self.client.send_goal(*speeds)
            self.last_cmd = speeds
            print(f"按下{k.upper()}，速度: {speeds}")
        if k == 'q':
            print("已停止小车。")
        if k == '\x1b':  # ESC退出
            print("退出键盘控制。")
            return False

    def run(self):
        with keyboard.Listener(on_press=self.on_press) as listener:
            listener.join()

if __name__ == "__main__":
    ctrl = KeyboardControl()
    try:
        ctrl.run()
    except KeyboardInterrupt:
        print("退出键盘控制。")
    finally:
        # 停止小车
        ctrl.client.send_goal(0.0, 0.0) 
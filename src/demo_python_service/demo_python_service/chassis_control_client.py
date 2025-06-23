#!/usr/bin/env python3 
import serial
import argparse
import threading
import time
import json

# 用于控制接收线程是否输出
print_lock = threading.Lock()
ser = None

class ChassisSerialClient:
    def __init__(self, port):
        global ser
        try:
            ser = serial.Serial(port, baudrate=115200, timeout=1)
            ser.setRTS(False)
            ser.setDTR(False)
            self.get_logger().info(f"串口 {port} 连接成功")
            # 启动接收线程
            self.recv_thread = threading.Thread(target=self.read_serial)
            self.recv_thread.daemon = True
            self.recv_thread.start()
        except serial.SerialException as e:
            self.get_logger().error(f"无法打开串口 {port}: {e}")
            raise

    def get_logger(self):
        class Logger:
            @staticmethod
            def info(msg):
                print(f"[INFO] {msg}")
            @staticmethod
            def error(msg):
                print(f"[ERROR] {msg}")
        return Logger()
    def send_goal(self, left_speed, right_speed):
            """发送底盘控制命令到串口
            left_speed: 左侧轮速度(m/s)，范围-0.5~0.5
            right_speed: 右侧轮速度(m/s)，范围-0.5~0.5
            """
            if ser is None or not ser.is_open:
                self.get_logger().error("串口未连接，无法发送命令")
                return

            # 根据UGV02指令集，使用CMD_SPEED_CTRL命令(T=1)
            cmd = {
                "T": 1,  # 左右轮速度控制指令
                "L": left_speed,  # 左侧轮速度(m/s)
                "R": right_speed  # 右侧轮速度(m/s)
            }
            # 限制速度在有效范围内(-0.5 ~ +0.5)
            cmd["L"] = max(min(cmd["L"], 0.5), -0.5)
            cmd["R"] = max(min(cmd["R"], 0.5), -0.5)

            try:
                command_str = json.dumps(cmd) + "\n"
                ser.write(command_str.encode('utf-8'))
                self.get_logger().info(f"已发送命令: {command_str.strip()}")
            except Exception as e:
                self.get_logger().error(f"发送命令失败: {str(e)}")
    def read_serial(self):
        """读取串口数据的线程函数"""
        while True:
            try:
                with print_lock:
                    data = ser.readline().decode('utf-8').strip()
                    if data:
                        try:
                            # 解析JSON格式的反馈数据
                            feedback = json.loads(data)
                            self.get_logger().info(f"收到反馈: 距离={feedback.get('distance', 0):.2f}m, 状态={feedback.get('status', 'unknown')}")
                        except json.JSONDecodeError:
                            self.get_logger().info(f"收到数据: {data}")
            except Exception as e:
                self.get_logger().error(f"串口读取错误: {str(e)}")
                break

def main(): 
    try:
        # 创建底盘串口客户端，使用指定的串口名称
        client = ChassisSerialClient("/dev/car_controller")
        # 示例：控制机器人以 0.1 m/s 的线速度，0 rad/s 的角速度移动 10 秒
        # 示例：控制机器人前进（左右轮速度均为0.1m/s）
            client.send_goal(0.1, 0.1)
    except Exception as e:
        print(f"程序异常退出: {str(e)}")
    finally:
        if ser and ser.is_open:
            ser.close()
            print("串口已关闭")

if __name__ == '__main__': 
    main()
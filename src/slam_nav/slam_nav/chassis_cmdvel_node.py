#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import threading
import json
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import math
import time
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

# 如果没有tf_transformations库，可以用以下函数将欧拉角转四元数：
def euler_to_quaternion(yaw, pitch=0.0, roll=0.0):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

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
                timeout=1
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

        # 里程计相关变量
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now().seconds_nanoseconds()[0] + \
                         self.get_clock().now().seconds_nanoseconds()[1] * 1e-9
        self.vx = 0.0
        self.vth = 0.0
        # 里程计发布器
        self.odom_pub = self.create_publisher(Odometry, '/odom', 20)
        # tf变换发布器
        self.tf_broadcaster = TransformBroadcaster(self)
        # 定时器，定期发布odom
        self.odom_timer = self.create_timer(0.02, self.publish_odom)  # 50Hz

    def cmd_vel_callback(self, msg):
        """处理速度指令消息"""
        # 差速模型转换，左右轮速度，单位m/s，范围-0.5~0.5
        left_speed = max(min(msg.linear.x - msg.angular.z, 0.5), -0.5)
        right_speed = max(min(msg.linear.x + msg.angular.z, 0.5), -0.5)
        # 推荐：CMD_SPEED_CTRL 指令
        cmd = {"T": 1, "L": left_speed, "R": right_speed}
        # # 如需使用CMD_ROS_CTRL指令（T=13），请注释上面一行，取消下面三行注释：
        # cmd = {"T": 13, "X": msg.linear.x, "Z": msg.angular.z}
        # # 或PWM控制（T=11），请自行映射速度到PWM值
        # # cmd = {"T": 11, "L": int(left_speed * 255 / 0.5), "R": int(right_speed * 255 / 0.5)}
        command_str = json.dumps(cmd) + "\n"
        
        # 保存线速度和角速度用于里程计推算
        self.vx = msg.linear.x
        self.vth = msg.angular.z
        # 发送到串口
        try:
            self.ser.write(command_str.encode('utf-8'))
            self.get_logger().info(f"串口已发送: {command_str.strip()}")
        except Exception as e:
            self.get_logger().error(f"串口写入错误: {e}")
    
    def read_serial(self):
        """持续读取串口数据的线程函数"""
        while rclpy.ok():
            try:
                if self.ser.in_waiting:
                    data = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    self.get_logger().info(f"串口收到: {data}")
            except Exception as e:
                self.get_logger().error(f"串口读取错误: {e}")
                break
    
    def cleanup(self):
        """关闭串口"""
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("串口已关闭")

    def publish_odom(self):
        """定时发布里程计信息"""
        now = self.get_clock().now().seconds_nanoseconds()[0] + \
              self.get_clock().now().seconds_nanoseconds()[1] * 1e-9
        dt = now - self.last_time
        self.last_time = now

        # 基于上一时刻速度积分推算位姿
        delta_x = self.vx * math.cos(self.th) * dt
        delta_y = self.vx * math.sin(self.th) * dt
        delta_th = self.vth * dt
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = euler_to_quaternion(self.th)
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.vth
        self.odom_pub.publish(odom)
        self.get_logger().info(f"已发布里程计: {odom}")

        # 发布tf变换（odom->base_footprint）
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = euler_to_quaternion(self.th)
        self.tf_broadcaster.sendTransform(t)

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
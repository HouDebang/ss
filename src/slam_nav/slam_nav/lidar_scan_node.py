#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial
import threading
import math
import time

class LidarScanNode(Node):
    def __init__(self):
        super().__init__('lidar_scan_node')
        self.declare_parameter('port', '/dev/Lidar')
        self.declare_parameter('baudrate', 115200)
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        try:
            self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=1)
            self.get_logger().info(f"成功连接到激光雷达串口: {port}")
        except serial.SerialException as e:
            self.get_logger().error(f"无法打开激光雷达串口: {e}")
            rclpy.shutdown()
            return
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.get_logger().info(f"成功创建激光雷达发布者: /scan")
        self.running = True
        self.lidar_thread = threading.Thread(target=self.read_lidar)
        self.lidar_thread.daemon = True
        self.lidar_thread.start()

    def read_lidar(self):
        while rclpy.ok() and self.running:
            try:
                # 这里假设每次读取一帧数据，具体协议需根据实际雷达型号修改
                # 示例：假设每帧为360个距离，单位米，按行文本输出
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                # 假设数据为以逗号分隔的360个距离值
                parts = line.split(',')
                if len(parts) != 360:
                    continue
                ranges = [float(x) if x else float('inf') for x in parts]
                scan = LaserScan()
                scan.header.stamp = self.get_clock().now().to_msg()
                scan.header.frame_id = 'laser'
                scan.angle_min = 0.0
                scan.angle_max = 2 * math.pi
                scan.angle_increment = 2 * math.pi / 360
                scan.time_increment = 0.0
                scan.scan_time = 0.1  # 假设10Hz
                scan.range_min = 0.05
                scan.range_max = 12.0
                scan.ranges = ranges
                scan.intensities = []
                self.scan_pub.publish(scan)
            except Exception as e:
                self.get_logger().error(f"激光雷达读取错误: {e}")
                time.sleep(0.1)

    def cleanup(self):
        self.running = False
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("激光雷达串口已关闭")


def main(args=None):
    rclpy.init(args=args)
    node = LidarScanNode()
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
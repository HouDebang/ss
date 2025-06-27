#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial
import threading
import struct
import math
import time

class LD19LidarNode(Node):
    PKG_HEADER = 0x54
    PKG_VER_LEN = 0x2C
    POINT_PER_PACK = 12
    FRAME_SIZE = 47  # 每帧47字节
    CRC_TABLE = [
        0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33,
        0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5, 0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b,
        0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43, 0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44,
        0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b,
        0xb5, 0xf8, 0x2f, 0x62, 0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86,
        0xcb, 0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5,
        0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4, 0x7c, 0x31, 0xe6, 0xab, 0x05,
        0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20, 0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b,
        0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b,
        0x46, 0xe8, 0xa5, 0x72, 0x3f, 0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c,
        0xdb, 0x96, 0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e, 0xeb,
        0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7, 0x5d, 0x10, 0xc7, 0x8a,
        0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01, 0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17,
        0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
    ]

    def __init__(self):
        super().__init__('ld19_lidar_node')
        self.declare_parameter('port', '/dev/Lidar')
        self.declare_parameter('baudrate', 230400)
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.ser = serial.Serial(port, baudrate, timeout=1)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.frame_id = 'base_lidar_link'
        self.running = True
        self.buffer = bytearray()
        self.one_circle = []  # 存储一圈点
        self.last_start_angle = None
        self.lidar_thread = threading.Thread(target=self.read_lidar)
        self.lidar_thread.daemon = True
        self.lidar_thread.start()

    def calc_crc(self, data):
        crc = 0
        for b in data[:-1]:
            crc = self.CRC_TABLE[(crc ^ b) & 0xff]
        return crc

    def read_lidar(self):
        while self.running:
            self.buffer += self.ser.read(512)
            while len(self.buffer) >= self.FRAME_SIZE:
                # 查找帧头
                if self.buffer[0] == self.PKG_HEADER and self.buffer[1] == self.PKG_VER_LEN:
                    self.get_logger().info("找到帧头，准备解析一帧")
                    frame = self.buffer[:self.FRAME_SIZE]
                    self.buffer = self.buffer[self.FRAME_SIZE:]
                    # CRC校验
                    if self.calc_crc(frame) == frame[-1]:
                        self.get_logger().info("CRC校验通过，解析帧数据")
                        # 解析数据帧
                        pkg = self.parse_frame(frame)
                        if pkg:
                            self.one_circle.extend(pkg)
                            # 检查是否转了一圈（角度回绕）
                            if self.last_start_angle is not None and pkg[0]['angle'] < self.last_start_angle:
                                self.publish_scan()
                                self.one_circle = pkg
                            self.last_start_angle = pkg[0]['angle']

    def parse_frame(self, frame):
        # 结构参考官方协议
        # 帧头2字节，帧长1字节，速度2字节，起始角2字节，点数据(12*3字节)，结束角2字节，时间戳4字节，CRC1字节
        speed = struct.unpack('<H', frame[2:4])[0] / 100.0  # rpm
        start_angle = struct.unpack('<H', frame[4:6])[0] / 100.0
        end_angle = struct.unpack('<H', frame[42:44])[0] / 100.0
        points = []
        for i in range(self.POINT_PER_PACK):
            offset = 6 + i * 3
            distance = struct.unpack('<H', frame[offset:offset+2])[0] / 1000.0  # mm->m
            confidence = frame[offset+2]
            angle = start_angle + (end_angle - start_angle) * i / (self.POINT_PER_PACK - 1)
            if angle < 0:
                angle += 360.0
            elif angle >= 360.0:
                angle -= 360.0
            points.append({'angle': angle, 'distance': distance, 'confidence': confidence})
        return points

    def publish_scan(self):
        # 按角度排序
        points = sorted(self.one_circle, key=lambda x: x['angle'])
        ranges = [float('inf')] * 360
        intensities = [0.0] * 360
        for pt in points:
            idx = int(pt['angle']) % 360
            if pt['distance'] > 0.03 and pt['distance'] < 12.0:
                ranges[idx] = pt['distance']
                intensities[idx] = pt['confidence']
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.frame_id
        scan.angle_min = 0.0
        scan.angle_max = 2 * math.pi
        scan.angle_increment = 2 * math.pi / 360
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.03
        scan.range_max = 12.0
        scan.ranges = ranges
        scan.intensities = intensities
        self.scan_pub.publish(scan)
        self.get_logger().info("发布一圈/scan")

    def cleanup(self):
        self.running = False
        if self.ser.is_open:
            self.ser.close()


def main(args=None):
    rclpy.init(args=args)
    node = LD19LidarNode()
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
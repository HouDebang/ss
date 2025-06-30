import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import json
import math

class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.declare_parameter('port', '/dev/car_controller')
        self.declare_parameter('baudrate', 115200)
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            self.get_logger().info(f"成功连接到串口: {port}")
        except serial.SerialException as e:
            self.get_logger().error(f"无法打开串口: {e}")
            rclpy.shutdown()
            return
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)
        self.timer = self.create_timer(0.05, self.get_and_publish_imu)  # 20Hz

    def get_and_publish_imu(self):
        try:
            # 发送获取IMU数据指令
            self.ser.write(b'{"T":126}\n')
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                return
            data = json.loads(line)
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "base_imu_link"
            # 以下字段请根据你的IMU协议实际字段名填写
            # 假设data包含四元数(qx,qy,qz,qw)、角速度(ax,ay,az)、线加速度(lx,ly,lz)
            imu_msg.orientation.x = data.get('qx', 0.0)
            imu_msg.orientation.y = data.get('qy', 0.0)
            imu_msg.orientation.z = data.get('qz', 0.0)
            imu_msg.orientation.w = data.get('qw', 1.0)
            imu_msg.angular_velocity.x = data.get('gx', 0.0)
            imu_msg.angular_velocity.y = data.get('gy', 0.0)
            imu_msg.angular_velocity.z = data.get('gz', 0.0)
            imu_msg.linear_acceleration.x = data.get('ax', 0.0)
            imu_msg.linear_acceleration.y = data.get('ay', 0.0)
            imu_msg.linear_acceleration.z = data.get('az', 0.0)
            self.imu_pub.publish(imu_msg)
        except Exception as e:
            self.get_logger().warn(f"IMU数据解析失败: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'ser') and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

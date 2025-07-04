import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import json
import math
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

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
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_timer = self.create_timer(0.05, self.publish_imu_tf)  # 20Hz

    def get_and_publish_imu(self):
        try:
            self.ser.write(b'{"T":126}\n')
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                return
            # 只处理以 { 开头且以 } 结尾的内容
            if not (line.startswith('{') and line.endswith('}')):
                return
            try:
                data = json.loads(line)
            except Exception as e:
                self.get_logger().warn(f"IMU数据解析失败: {e}，原始数据: {line}")
                return

            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "base_imu_link"

            # 默认值
            imu_msg.orientation.x = 0.0
            imu_msg.orientation.y = 0.0
            imu_msg.orientation.z = 0.0
            imu_msg.orientation.w = 1.0

            # T=1001: 加速度/角速度
            if data.get('T') == 1001:
                imu_msg.linear_acceleration.x = float(data.get('ax', 0.0) or 0.0)
                imu_msg.linear_acceleration.y = float(data.get('ay', 0.0) or 0.0)
                imu_msg.linear_acceleration.z = float(data.get('az', 0.0) or 0.0)
                imu_msg.angular_velocity.x = float(data.get('gx', 0.0) or 0.0)
                imu_msg.angular_velocity.y = float(data.get('gy', 0.0) or 0.0)
                imu_msg.angular_velocity.z = float(data.get('gz', 0.0) or 0.0)
            # T=1002: 四元数
            elif data.get('T') == 1002:
                imu_msg.orientation.x = float(data.get('q1', 0.0) or 0.0)
                imu_msg.orientation.y = float(data.get('q2', 0.0) or 0.0)
                imu_msg.orientation.z = float(data.get('q3', 0.0) or 0.0)
                imu_msg.orientation.w = float(data.get('q0', 1.0) or 1.0)
            else:
                return

            self.imu_pub.publish(imu_msg)
            self.get_logger().info(f"已发布IMU数据: {imu_msg}")
        except Exception as e:
            self.get_logger().warn(f"IMU数据解析失败: {e}")

    def publish_imu_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"  # 发布的坐标系
        t.child_frame_id = "base_imu_link"
        t.transform.translation.x = 0.0  # 如有实际偏移请填写
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

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

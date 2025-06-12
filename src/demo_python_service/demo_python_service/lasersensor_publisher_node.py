import rclpy 
from rclpy.node import Node
from std_msgs.msg import Float32 
import numpy as np
import serial
import time
def main(args=None): 
    rclpy.init(args=args) # 初始化
    try:
        lidar_ser=serial.Serial('/dev/wheeltec_controller',230400,timeout=1)
        print("[Lidar]已连接到/dev/wheeltec_controller")
    except Exception as e:
        print("[Lidar]已连接失败",e)
        exit(1)
    node = PublisherNode("lasersensor_publisher_node", lidar_ser) # 新建一个节点
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy

class PublisherNode(Node):
    def __init__(self,name,lidar_ser):
        super().__init__(name)
        self.get_logger().info(f"{name} Created!")
        self.publisher_ = self.create_publisher(Float32,"laserdata_topic",10)
        self.create_timer(1.0, self.timer_callback)
        self.lidar_ser = lidar_ser

    def timer_callback(self):
        msg = Float32()
        distance = get_lidar_measurement(self.lidar_ser)
        msg.data = distance
        self.publisher_.publish(msg)
        self.get_logger().info(f'{self.get_name()} publishing: {msg.data}')#日志输出

def get_lidar_measurement(ser):
    raw_data=b""
    while True:
        if ser.in_waiting>0:
            raw_data+=ser.read(ser.in_waiting)
        start_index=raw_data.find(b'\xAA')
        if start_index!=-1 and len(raw_data)-start_index>=195:
            packet=raw_data[start_index:start_index+195]
            raw_data=raw_data[start_index+195:]
            measurements=[]
            data_list=list(packet)
        for i in range(10,195,15):
            if i+1<len(data_list):
                high=data_list[i+1]
                low=data_list[i]
                distance=(high<<8)|low
                measurements.append(distance)
        if measurements:
            average_distance=sum(measurements)/len(measurements)
            return average_distance
        time.sleep(0.05)


if __name__ == "__main__":
    main()
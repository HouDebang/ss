import rclpy 
from rclpy.node import Node
from std_msgs.msg import Float32 
import numpy as np

class SubscriberNode(Node): 
    def __init__(self,name): 
        super().__init__(name) #初始化父类
        self.get_logger().info (f"{name} Created!")#节点创建，日志输出
        self.create_subscription(Float32,"laserdata_topic",self.subscriber_callback,10) #创建一个订阅者
    def subscriber_callback(self,msg):
        self.get_logger().info(f'距离为{msg.data}')#日志输出




def main(args=None): 
    rclpy.init(args=args) # 初始化
    node = SubscriberNode("lasersensor_subscriber_node") # 新建一个节点
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy
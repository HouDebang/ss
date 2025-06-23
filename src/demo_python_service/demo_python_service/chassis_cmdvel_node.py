import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from chassis_control_client import ChassisSerialClient

class ChassisCmdVelNode(Node):
    def __init__(self):
        super().__init__('chassis_cmdvel_node')
        self.client = ChassisSerialClient("/dev/car_controller")
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.get_logger().info('已启动，订阅/cmd_vel')

    def cmd_vel_callback(self, msg):
        # Twist: linear.x, angular.z
        linear = msg.linear.x
        angular = msg.angular.z
        # 简单差速驱动模型
        left_speed = linear - angular
        right_speed = linear + angular
        self.client.send_goal(left_speed, right_speed)
        self.get_logger().info(f"收到/cmd_vel: linear={linear:.2f}, angular={angular:.2f} -> L={left_speed:.2f}, R={right_speed:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = ChassisCmdVelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('退出chassis_cmdvel_node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main() 
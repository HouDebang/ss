#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node 
from rclpy.action import ActionClient 
import json
#  导入自定义动作接口
from chapt4_interfaces.action import ChassisControl
class ChassisActionClient(Node): 
    def __init__(self): 
        super().__init__('chassis_action_client') 
        self._action_client = ActionClient(self, ChassisControl, 'chassis_control')
    def send_goal(self, ip_address, linear_velocity, angular_velocity, duration): 
        goal_msg = ChassisControl.Goal() 
        goal_msg.ip_address = ip_address 
        goal_msg.linear_velocity = linear_velocity 
        goal_msg.angular_velocity = angular_velocity 
        goal_msg.duration = duration
        self.get_logger().info("Waiting for action server...") 
        self._action_client.wait_for_server() 
        self.get_logger().info("Action server available. Sending goal...")
        send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)
    #执行过程中的回调
    def feedback_callback(self, feedback_msg): 
        feedback = feedback_msg.feedback 
        self.get_logger().info(f"Feedback: Distance travelled = {feedback.distance_travelled:.2f} m")
    #收到回应后的回调
    def goal_response_callback(self, future): 
        goal_handle = future.result() 
        if not goal_handle.accepted: 
            self.get_logger().info("Goal rejected.") 
            return 
        self.get_logger().info("Goal accepted. Waiting for result...")
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)
    #执行完成后的回调
    def get_result_callback(self, future): 
        result = future.result().result 
        self.get_logger().info(f"Result received: success = {result.success}, message: {result.message}") 
        rclpy.shutdown()

def main(args=None): 
    rclpy.init(args=args) 
    client = ChassisActionClient() 
    # 示例目标：控制机器人以 0.1 m/s 的线速度，0 rad/s 的角速度移动 10 秒，目标 IP 为 192.168.4.1 
    client.send_goal("192.168.4.1", 0.1, 0.0, 10.0) 
    rclpy.spin(client) 
if __name__ == '__main__': 
    main()
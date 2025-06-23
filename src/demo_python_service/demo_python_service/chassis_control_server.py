#!/usr/bin/env python3 
import rclpy 
from rclpy.node import Node 
from rclpy.action import ActionServer 
import json 
import requests 
import time 
# 导入自定义动作接口
from chapt4_interfaces.action import ChassisControl
class ChassisActionServer(Node): 
    def __init__(self): 
        super().__init__('chassis_action_server')
        self._action_server = ActionServer( 
            self,
            ChassisControl, 
            'chassis_control', 
            self.execute_callback 
            )
        self.get_logger().info("Chassis Action Server started.")

    async def execute_callback(self, goal_handle):
        #接收到的请求
        goal = goal_handle.request
        ip_addr = goal.ip_address 
        linear_velocity = goal.linear_velocity 
        angular_velocity = goal.angular_velocity 
        duration = goal.duration 
        self.get_logger().info( f"Received goal: linear_velocity={linear_velocity}, angular_velocity={angular_velocity}, " 
        f"duration={duration}, ip_address={ip_addr}" 
        )
        segment_time = 1.0  # 每段执行 1 秒 
        segments = int(duration // segment_time) 
        remaining = duration - segments * segment_time 
        total_distance = 0.0 
        #每秒进行反馈
        for i in range(segments): 
            # 构造 JSON 指令：CMD_ROS_CTRL 指令 T=13
            command = {"T": 13, "X": linear_velocity, "Z": angular_velocity} 
            url = f"http://{ip_addr}/js?json=" + json.dumps(command) 
            try: 
                response = requests.get(url, timeout=1) 
                self.get_logger().info("Sent command, response: " + response.text) 
            except Exception as e: 
                self.get_logger().error("HTTP command failed: " + str(e)) 
                goal_handle.abort() 
                return ChassisControl.Result(success=False, message="HTTP command failed") 
                #时间反馈
            time.sleep(segment_time) 
            total_distance += abs(linear_velocity) * segment_time
            feedback = ChassisControl.Feedback() 
            feedback.distance_travelled = total_distance 
            goal_handle.publish_feedback(feedback)
        # 处理剩余时间
        if remaining > 0: 
            command = {"T": 13, "X": linear_velocity, "Z": angular_velocity} 
            url = f"http://{ip_addr}/js?json=" + json.dumps(command) 
            try: 
                response = requests.get(url, timeout=1) 
                self.get_logger().info("Sent command, response: " + response.text) 
            except Exception as e: 
                self.get_logger().error("HTTP command failed: " + str(e)) 
                goal_handle.abort()
                return ChassisControl.Result(success=False, message="HTTP command failed")
            time.sleep(remaining) 
            total_distance += abs(linear_velocity) * remaining 
            feedback = ChassisControl.Feedback() 
            feedback.distance_travelled = total_distance 
            goal_handle.publish_feedback(feedback) 
        goal_handle.succeed()
        result = ChassisControl.Result()
        result.success = True 
        result.message = f"Movement executed. Total distance travelled: {total_distance:.2f} m." 
        return result
def main(args=None): 
    rclpy.init(args=args) 
    node = ChassisActionServer() 
    rclpy.spin(node) 
    node.destroy_node() 
    rclpy.shutdown() 
if __name__ == '__main__': 
    main()
import rclpy
from rclpy.node import Node
from chapt4_interfaces.srv import FaceDetector
from chapt4_interfaces.msg import FacePosition
from cv_bridge import CvBridge
import cv2
import os
import sys
sys.path.append(os.path.expanduser("~/face_recognition_evn/lib/python3.12/site-packages"))
import face_recognition
from ament_index_python.packages import get_package_share_directory
import time


class FaceDetectNode(Node):
    def __init__(self):
        super().__init__('face_detect_node')
        self.srv = self.create_service(FaceDetector, 'face_detect', self.face_detect_callback)
        self.position_publisher = self.create_publisher(
            FacePosition, 
            'face_positions', 
            10
        )
        self.get_logger().info("人脸位置信息将发布到 /face_positions 话题")
        self.bridge = CvBridge()
        self.number_of_times_to_upsample=1
        self.model="hog"
        self.default_image_path= get_package_share_directory('demo_python_service') + "/resource/default.jpg"
        self.get_logger().info("Face detect node has been started")

    def face_detect_callback(self, request, response):
        if request.image.data:
            cv_image = self.bridge.imgmsg_to_cv2(request.image)
        else:
            cv_image = cv2.imread(self.default_image_path)
            self.get_logger().info("No image received, using default image")
        start_time=time.time()
        self.get_logger().info("Start face detection")
        face_locations = face_recognition.face_locations(cv_image,number_of_times_to_upsample=self.number_of_times_to_upsample,model=self.model)
        response.use_time=time.time()-start_time
        response.number=len(face_locations)
        for top, right, bottom, left in face_locations:
            response.top.append(top)
            response.right.append(right)
            response.bottom.append(bottom)
            response.left.append(left)
        
        position_msg = FacePosition()
        position_msg.number = response.number
        position_msg.top = response.top
        position_msg.right = response.right
        position_msg.bottom = response.bottom  # 修正变量名
        position_msg.left = response.left
        position_msg.use_time = time.time()-start_time
        
        self.position_publisher.publish(position_msg)
        self.get_logger().info(f"已发布人脸位置信息：检测到 {response.number} 个人脸")
        return response
def main():
    rclpy.init()
    node=FaceDetectNode()
    rclpy.spin(node)
    rclpy.shutdown()
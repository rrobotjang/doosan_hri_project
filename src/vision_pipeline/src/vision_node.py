#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
# YOLO 모델 로드 및 추론 라이브러리 (예: ultralytics) 임포트 필요

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/object_detection/pose', 10)
        self.latest_depth_image = None
        
        # YOLO 모델 로드 (여기서는 플레이스홀더)
        self.yolo_model = yolo8v.pt

    def depth_callback(self, msg):
        self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def image_callback(self, msg):
        if self.latest_depth_image is None:
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # 1. YOLO/OpenCV를 사용한 물체 탐지 (바운딩 박스, 키포인트)
        # results = self.yolo_model(cv_image)
        # detected_object = results.pred[0] # 예시

        # 2. 뎁스 이미지와 융합하여 3D 위치 추정
        # 카메라 캘리브레이션 파라미터 K 필요
        # px, py, bbox_center_x, bbox_center_y = ...
        # depth = self.latest_depth_image[py, px]
        # X = depth * (px - cx) / fx
        # Y = depth * (py - cy) / fy
        # Z = depth / fz
        
        # 3. 6D Pose 메시지 발행 (테스트용 더미 데이터)
        object_pose_msg = PoseStamped()
        object_pose_msg.header.stamp = self.get_clock().now().to_msg()
        object_pose_msg.header.frame_id = "camera_link" # 또는 카메라 프레임 ID
        object_pose_msg.pose.position.x = 0.5
        object_pose_msg.pose.position.y = 0.1
        object_pose_msg.pose.position.z = 0.05
        # 방향 (orientation) 설정 필요
        self.pose_pub.publish(object_pose_msg)

        # 4. RViz 시각화를 위해 처리된 이미지 발행 (선택 사항)
        # moveit_visual_tools 또는 자체 마커 발행 노드를 사용하여 RViz에 시각화

def main(args=None):
    rclpy.init(args=args)
    vision_node = VisionNode()
    rclpy.spin(vision_node)
    vision_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

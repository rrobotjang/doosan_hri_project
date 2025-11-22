#vision_node와 pick_place_controller의 기능을 하나의 Python 스크립트 내에서 구현하며, ROS 2의 메커니즘을 사용하여 효율적으로 연동

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Float64, String
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import json

# MoveIt! Python 바인딩 또는 인터페이스는 백그라운드 스레드에서 처리
# from moveit_configs_utils import MoveGroupPythonInterface # ROS 2 Foxy/Galactic 기준 예시

class IntegratedRobotSystem(Node):
    def __init__(self):
        super().__init__('integrated_robot_system')
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        
        # --- 비전 관련 설정 (포그라운드 워커) ---
        self.image_sub = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/object_detection/pose', 10)
        self.latest_depth_image = None
        self.latest_color_image = None
        
        # --- 컨트롤러 관련 설정 (백그라운드 워커) ---
        self.gripper_pub = self.create_publisher(Float64, '/gripper_controller/command', 10)
        self.hri_sub = self.create_subscription(String, '/hri/command_json', self.hri_command_callback, 10)
        self.target_object_pose = None
        self.task_state = "IDLE" # IDLE, DETECTING, PLANNING, EXECUTING
        
        # MoveIt! 인터페이스 초기화 (더미 플레이스홀더)
        # self.move_group = MoveGroupPythonInterface(self) 

    # --- 포그라운드: 비전 콜백 함수들 ---
    def depth_callback(self, msg):
        with self.lock:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def image_callback(self, msg):
        with self.lock:
            self.latest_color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        if self.latest_depth_image is not None and self.task_state == "DETECTING":
            self.process_vision_data()

    def process_vision_data(self):
        # YOLO/OpenCV 처리 및 6D Pose 계산 로직 (VisionNode 코드 참고)
        # ...
        
        # 계산된 Pose 업데이트 및 발행
        object_pose_msg = PoseStamped()
        object_pose_msg.header.stamp = self.get_clock().now().to_msg()
        object_pose_msg.header.frame_id = "camera_link"
        object_pose_msg.pose.position.x = 0.5
        object_pose_msg.pose.position.y = 0.1
        object_pose_msg.pose.position.z = 0.05
        self.pose_pub.publish(object_pose_msg)
        
        # 백그라운드 워커에게 새 목표 알림
        with self.lock:
            self.target_object_pose = object_pose_msg.pose
            self.task_state = "PLANNING"
        self.get_logger().info("Object detected. Switching state to PLANNING.")

    # --- 백그라운드: 컨트롤러 및 HRI 콜백 함수들 ---
    def hri_command_callback(self, msg):
        """
        STT/LLM 파이프라인에서 받은 JSON 명령어를 처리합니다.
        """
        try:
            command_data = json.loads(msg.data)
            action = command_data.get("command")

            if action == "start_pick_place" and self.task_state == "IDLE":
                self.get_logger().info("HRI Command: Start detecting objects.")
                with self.lock:
                    self.task_state = "DETECTING"
            elif action == "go_home":
                # self.move_group.go_to_home()
                pass
            # ... 기타 HRI 명령 처리 ...
            
        except json.JSONDecodeError:
            self.get_logger().error(f"Failed to decode JSON command: {msg.data}")

    def background_worker_loop(self):
        """
        MoveIt! 계획 및 실행은 비동기적으로 수행될 수 있도록 별도 루프 또는 콜백에서 관리
        """
        while rclpy.ok():
            if self.task_state == "PLANNING":
                with self.lock:
                    target_pose = self.target_object_pose
                    self.task_state = "EXECUTING"
                
                self.get_logger().info("Starting execution in background worker.")
                # self.execute_pick_and_place(target_pose) # MoveIt! 실행 함수 호출
                
                with self.lock:
                    self.task_state = "IDLE"
                    self.get_logger().info("Execution complete. Back to IDLE.")
            
            # 짧은 슬립으로 CPU 부하 조절
            import time
            time.sleep(0.1)

    # MoveIt 실행 로직 (이전 코드의 execute_pick_and_place 함수)
    # ...

def main(args=None):
    rclpy.init(args=args)
    integrated_system = IntegratedRobotSystem()
    
    # 백그라운드 워커 스레드 시작
    worker_thread = threading.Thread(target=integrated_system.background_worker_loop)
    worker_thread.daemon = True
    worker_thread.start()

    # ROS 2 멀티스레드 실행자로 모든 콜백 처리 (저지연 핵심)
    executor = MultiThreadedExecutor()
    executor.add_node(integrated_system)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    
    integrated_system.destroy_node()
    rclpy.shutdown()
    worker_thread.join() # 스레드 종료 대기

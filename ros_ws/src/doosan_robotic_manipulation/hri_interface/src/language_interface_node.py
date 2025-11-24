#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class LanguageInterfaceNode(Node):
    def __init__(self):
        super().__init__('language_interface_node')
        # FastAPI 서버가 발행하는 토픽 구독
        self.subscription = self.create_subscription(
            String,
            '/hri/command_json',
            self.listener_callback,
            10)
        # 실제 로봇 컨트롤러가 HRI 명령을 구독할 수 있도록 토픽 발행 (옵션)
        # self.hri_pub = self.create_publisher(String, '/robot_internal/hri_goal', 10)

    def listener_callback(self, msg):
        # 메시지를 받으면 main_controller로 전달되며, 여기서는 로깅만 수행
        self.get_logger().info(f'Received JSON command from API: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = LanguageInterfaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

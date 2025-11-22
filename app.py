# app.py
from flask import Flask, request, jsonify
import torch
import json
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from transformers import pipeline, AutoTokenizer, AutoModelForCausalLM, BitsAndBytesConfig
from peft import PeftModel
import threading

app = Flask(__name__)

# --- STT 및 LLM 모델 초기화 (이전 코드 참고) ---
# 이 부분은 서버 시작 시 한 번 로드됩니다.
# 모델 로드 코드를 여기에 삽입... (예시)
# whisper_pipeline = pipeline("automatic-speech-recognition", model="openai/whisper-large-v3", device="cuda:0")
# model, tokenizer = load_llama_model_and_tokenizer() # 별도 함수로 캡슐화 필요

# ROS 2 Publisher 설정
class CommandPublisher(Node):
    def __init__(self):
        super().__init__('command_api_publisher')
        # integrated_robot_system의 HRI 구독 토픽과 동일해야 함
        self.publisher_ = self.create_publisher(String, '/hri/command_json', 10)

rclpy.init(args=None)
node = CommandPublisher()
executor = rclpy.executors.MultiThreadedExecutor()
executor.add_node(node)
# ROS 스핀을 위한 별도 스레드 시작
threading.Thread(target=executor.spin, daemon=True).start()

def speech_to_json_command(audio_file_path):
    # (이전 코드의 speech_to_json_command 함수 로직 사용)
    # STT 및 LLM 추론 결과를 반환
    transcribed_text = "테이블 위에 있는 빨간 공을 집어줘." # 더미 STT 결과
    json_string = "{\"command\": \"start_pick_place\", \"object\": \"red ball\"}" # 더미 LLM 결과
    return json_string

@app.route('/command', methods=['POST'])
def handle_command():
    if 'audio' not in request.files:
        return jsonify({"error": "No audio file provided"}), 400
    
    audio_file = request.files['audio']
    file_path = f"/tmp/{audio_file.filename}"
    audio_file.save(file_path)

    # STT 및 LLM 파이프라인 실행
    json_command_str = speech_to_json_command(file_path)
    os.remove(file_path) # 파일 정리

    if json_command_str:
        # ROS 2 토픽으로 JSON 발행
        msg = String()
        msg.data = json_command_str
        node.publisher_.publish(msg)
        return jsonify({"status": "success", "command": json.loads(json_command_str)}), 200
    else:
        return jsonify({"status": "failed", "message": "Could not generate valid JSON command"}), 500

if __name__ == '__main__':
    # 모델 로드 및 ROS 2 노드 초기화가 완료된 후 서버 실행
    app.run(host='0.0.0.0', port=5000)

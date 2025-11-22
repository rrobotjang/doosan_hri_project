# fastapi_app.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import json
import os
import shutil
from fastapi import FastAPI, UploadFile, File, HTTPException
from fastapi.responses import JSONResponse
# Hugging Face 라이브러리는 컨테이너 환경에 설치되어 있다고 가정
# from transformers import pipeline, AutoTokenizer, AutoModelForCausalLM, BitsAndBytesConfig
# from peft import PeftModel

app = FastAPI()

# --- ROS 2 Publisher 설정 (이전 코드와 동일) ---
class CommandPublisher(Node):
    def __init__(self):
        super().__init__('command_api_publisher')
        self.publisher_ = self.create_publisher(String, '/hri/command_json', 10)

rclpy.init(args=None)
ros_node = CommandPublisher()
executor = rclpy.executors.MultiThreadedExecutor()
executor.add_node(ros_node)
# ROS 스핀을 위한 별도 스레드 시작
threading.Thread(target=executor.spin, daemon=True).start()

# --- STT/LLM 함수 (더미 구현, 실제 모델 로드 필요) ---
def speech_to_json_command(audio_file_path):
    # 실제 모델 추론 로직 구현 필요
    # whisper_pipeline(...)
    # llama_model(...)
    print(f"Processing audio file: {audio_file_path}")
    transcribed_text = "테이블 위에 있는 빨간 공을 집어줘." 
    json_string = "{\"command\": \"start_pick_place\", \"object\": \"red ball\"}"
    return json_string

@app.post("/command")
async def handle_command(audio: UploadFile = File(...)):
    if not audio.filename.endswith(('.wav', '.mp3', '.ogg')):
        raise HTTPException(status_code=400, detail="Invalid file type.")

    # 임시 파일 저장
    file_path = f"/tmp/{audio.filename}"
    with open(file_path, "wb") as buffer:
        shutil.copyfileobj(audio.file, buffer)

    # STT 및 LLM 파이프라인 실행
    json_command_str = speech_to_json_command(file_path)
    os.remove(file_path)

    if json_command_str:
        # ROS 2 토픽으로 JSON 발행
        msg = String()
        msg.data = json_command_str
        ros_node.publisher_.publish(msg)
        return JSONResponse(status_code=200, content={"status": "success", "command": json.loads(json_command_str)})
    else:
        raise HTTPException(status_code=500, detail="Could not generate valid JSON command.")

# 애플리케이션 실행 명령어: uvicorn fastapi_app:app --host 0.0.0.0 --port 5000

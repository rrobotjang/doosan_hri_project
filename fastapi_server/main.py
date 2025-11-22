# fastapi_server/main.py
import uvicorn
from fastapi import FastAPI, File, UploadFile, HTTPException
from fastapi.responses import JSONResponse
import torch
import json
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from transformers import pipeline, AutoTokenizer, AutoModelForCausalLM, BitsAndBytesConfig
from peft import PeftModel
import threading
import shutil

app = FastAPI()

# --- STT 및 LLM 모델 로드 (이전 코드 참고) ---
# 실제 모델 로드 및 추론 함수는 여기에 구현되어야 합니다.
# 예시 함수 (더미 구현)
def speech_to_json_command(audio_file_path: str) -> str:
    # ... (Whisper 및 Llama 3 추론 로직) ...
    transcribed_text = "테이블 위에 있는 빨간 공을 집어줘." 
    json_string = "{\"command\": \"start_pick_place\", \"object\": \"red ball\"}" 
    return json_string

# --- ROS 2 Publisher 설정 ---
class CommandPublisher(Node):
    def __init__(self):
        super().__init__('command_api_publisher')
        self.publisher_ = self.create_publisher(String, '/hri/command_json', 10)

# ROS 2 초기화는 메인 앱 실행 전에 별도 스레드에서 처리
rclpy.init(args=None)
node = CommandPublisher()
executor = rclpy.executors.MultiThreadedExecutor()
executor.add_node(node)
threading.Thread(target=executor.spin, daemon=True).start()

@app.post("/command")
async def handle_command(audio: UploadFile = File(...)):
    # 오디오 파일 임시 저장
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
        node.publisher_.publish(msg)
        return JSONResponse(status_code=200, content={"status": "success", "command": json.loads(json_command_str)})
    else:
        raise HTTPException(status_code=500, detail="Could not generate valid JSON command")

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)

import torch
import json
from transformers import pipeline, AutoTokenizer, AutoModelForCausalLM, BitsAndBytesConfig
from peft import PeftModel

# --- 모델 불러오기 (학습 후 어댑터 적용) ---
# 실제 사용 시에는 학습된 어댑터를 불러와 기본 모델에 병합하거나 적용해야 합니다.

BASE_MODEL_ID = "meta-llama/Llama-2-7b-chat-hf" # 학습 시 사용한 기본 모델
ADAPTER_PATH = "./llama3_robot_adapter" # 학습된 LoRA 어댑터 경로

# 4비트 양자화 설정
bnb_config = BitsAndBytesConfig(
    load_in_4bit=True,
    bnb_4bit_quant_type="nf4",
    bnb_4bit_compute_dtype=torch.bfloat16,
)

# 기본 모델 불러오기
model = AutoModelForCausalLM.from_pretrained(BASE_MODEL_ID, quantization_config=bnb_config, device_map="auto")
tokenizer = AutoTokenizer.from_pretrained(BASE_MODEL_ID)

# LoRA 어댑터 적용
model = PeftModel.from_pretrained(model, ADAPTER_PATH)
model = model.merge_and_unload() # 메모리 여유가 있다면 병합하여 사용 가능
model.eval()

# --- STT 파이프라인 (Whisper) ---
# 한국어 번역이 포함된 Whisper large-v3 모델 사용
whisper_pipeline = pipeline("automatic-speech-recognition", model="openai/whisper-large-v3", device="cuda:0" if torch.cuda.is_available() else "cpu")

def speech_to_json_command(audio_file_path):
    """
    음성 파일 경로를 입력받아 STT 및 LLM 추론을 거쳐 JSON 명령을 반환합니다.
    """
    # 1. STT (Whisper를 이용한 한국어 인식 및 번역)
    # generate_kwargs={"language": "korean", "task": "translate"}를 사용하면 한국어로 번역합니다.
    # 한국어 음성을 인식만 하려면 task="transcribe"를 사용합니다.
    stt_result = whisper_pipeline(audio_file_path, generate_kwargs={"language": "korean"}) 
    transcribed_text = stt_result["text"]
    print(f"음성 인식 결과: {transcribed_text}")

    # 2. LLM 추론 (텍스트 -> JSON)
    # 모델에게 JSON 형식으로 출력하라는 명시적인 프롬프트를 제공합니다.
    prompt = f"다음 지시사항을 로봇 명령을 위한 JSON 형식으로 변환해줘: '{transcribed_text}'. 출력은 오직 유효한 JSON 문자열이어야 해."
    
    inputs = tokenizer(prompt, return_tensors="pt").to(model.device)
    with torch.no_grad():
        outputs = model.generate(
            **inputs, 
            max_new_tokens=100, 
            num_return_sequences=1, 
            eos_token_id=tokenizer.eos_token_id
        )
    
    generated_text = tokenizer.decode(outputs[0], skip_special_tokens=True)
    # 프롬프트 부분을 제거하고 순수 JSON 문자열만 추출하는 후처리 필요
    json_string = generated_text.replace(prompt, "").strip()

    try:
        # JSON 문자열을 Python 딕셔너리로 파싱
        command_json = json.loads(json_string)
        return command_json
    except json.JSONDecodeError:
        print(f"JSON 디코딩 실패: {json_string}")
        return None

# --- 사용 예시 ---
# audio_file_path = "path/to/your/audio_command.wav"
# command = speech_to_json_command(audio_file_path)
# if command:
#     print(f"로봇 컨트롤러로 전송될 JSON: {command}")

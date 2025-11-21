import torch
from datasets import load_dataset
from transformers import (
    AutoModelForCausalLM,
    AutoTokenizer,
    BitsAndBytesConfig,
    TrainingArguments,
    peft,
    sft,
    pipeline,
)
from peft import LoraConfig, PeftModel, prepare_model_for_kbit_training, get_peft_model
import os

# 1. 데이터셋 불러오기
# 데이터셋 형식에 맞춰 'instruction'과 'output' 컬럼을 사용합니다.
dataset = load_dataset("json", data_files="robot_commands.jsonl", split="train")

# 2. 모델 및 토크나이저 불러오기 (Llama 3 8B Instruct)
model_id = "meta-llama/Llama-2-7b-chat-hf" # Llama 3 8B는 허깅페이스에서 직접 불러올 때 사용자 인증이 필요할 수 있습니다. 
                                           # 접근 권한이 있다면 "meta-llama/Llama-3-8b-instruct"를 사용하세요.

# 4비트 양자화 설정 (int4)
bnb_config = BitsAndBytesConfig(
    load_in_4bit=True,
    bnb_4bit_quant_type="nf4",
    bnb_4bit_compute_dtype=torch.bfloat16,
    bnb_4bit_use_double_quant=True,
)

model = AutoModelForCausalLM.from_pretrained(
    model_id,
    quantization_config=bnb_config,
    device_map="auto",
    token=os.environ.get("HF_TOKEN") # Hugging Face 토큰 설정 필요
)
model.config.use_cache = False
model.config.pretraining_tp = 1

tokenizer = AutoTokenizer.from_pretrained(model_id, trust_remote_code=True)
tokenizer.pad_token = tokenizer.eos_token
tokenizer.padding_side = "right"

# 3. LoRA 설정 및 학습 준비
model = prepare_model_for_kbit_training(model)
lora_config = LoraConfig(
    r=16,
    lora_alpha=32,
    lora_dropout=0.05,
    bias="none",
    task_type="CAUSAL_LM",
    target_modules=["q_proj", "k_proj", "v_proj", "o_proj", "gate_proj", "up_proj", "down_proj"],
)
model = get_peft_model(model, lora_config)

# 4. SFT(Supervised Fine-Tuning) 학습 (실제 GPU 학습 환경에서 실행)
# 이 부분은 학습 서버에서 실행해야 합니다.
# training_arguments = TrainingArguments(
#     output_dir="./results",
#     per_device_train_batch_size=4,
#     gradient_accumulation_steps=4,
#     optim="paged_adamw_32bit",
#     save_steps=10,
#     logging_steps=10,
#     learning_rate=2e-4,
#     fp16=False,
#     bf16=True,
#     max_grad_norm=0.3,
#     num_train_epochs=1,
#     lr_scheduler_type="constant",
#     warmup_ratio=0.03,
# )

# trainer = sft.SFTTrainer(
#     model=model,
#     train_dataset=dataset,
#     peft_config=lora_config,
#     tokenizer=tokenizer,
#     args=training_arguments,
#     dataset_text_field="instruction", # 데이터셋 컬럼에 맞게 수정
#     max_seq_length=512,
# )

# trainer.train()
# trainer.save_model("llama3_robot_adapter")

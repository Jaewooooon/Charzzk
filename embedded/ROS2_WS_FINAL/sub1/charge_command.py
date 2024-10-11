from fastapi import FastAPI
from pydantic import BaseModel, Field
import json
import asyncio
import datetime
from message_producer import MessageProducer

app = FastAPI()

class ChargeCommandRequest(BaseModel):
    chargerId: int  
    reservationId: int 
    latitude: float 
    longitude: float 
    carNumber: str 
    duration: int 
    batteryLevel: int = Field(..., alias="battery") 

# JSON 파일에 데이터 저장 함수
def save_request_to_json(request: ChargeCommandRequest, filename="charge_command.json"):
    data = request.dict()  
    with open(filename, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False)
    print(f"Data saved to {filename}")

# 충전 로직 함수 -> 충전로봇이 움직일 수 있게 해야함.
def actual_charge_process(request: ChargeCommandRequest):
    # chargerId가 1일 때, 실제 충전 명령 로직을 처리하는 부분
    # 추후 로직 구현 예정
         
    # 요청 데이터를 JSON 파일에 저장
    save_request_to_json(request)
    return {"status": "success", "message": "Actual charge process initiated", "data": request}

# 가짜 충전 프로세스 함수
async def fake_charge_process(request: ChargeCommandRequest):
    producer = MessageProducer()

    # 5초 뒤에 '인식 성공' 메시지 전송
    await asyncio.sleep(5)
    recognition_status_message = {
        "type": "recognition_status",
        "status": "success",
        "reservationId": request.reservationId,
        "chargerId": request.chargerId,
        "carNumber": request.carNumber,
        "timestamp": datetime.datetime.now().isoformat()
    }
    producer.send_message_to_queue(recognition_status_message)
    print(f"Recognition Status Sent: {recognition_status_message}")

    # 충전 시작: 매 초마다 배터리 상태 메시지 전송
    battery_level = request.batteryLevel
    for second in range(request.duration):
        if battery_level < 100:
            battery_level += 1  # 매 초마다 배터리 레벨 1% 증가
        battery_status_message = {
            "type": "battery_status",
            "batteryLevel": battery_level,
            "reservationId": request.reservationId,
            "chargerId": request.chargerId,
            "carNumber": request.carNumber,
            "timestamp": datetime.datetime.now().isoformat()
        }
        producer.send_message_to_queue(battery_status_message)
        print(f"Battery Status Sent: {battery_status_message}")
        await asyncio.sleep(1)  # 1초마다 상태 업데이트

    # 충전 완료 메시지 전송
    charge_complete_message = {
        "type": "charge_complete",
        "reservationId": request.reservationId,
        "chargerId": request.chargerId,
        "carNumber": request.carNumber,
        "timestamp": datetime.datetime.now().isoformat()
    }
    producer.send_message_to_queue(charge_complete_message)
    print(f"Charge Complete Sent: {charge_complete_message}")

    producer.close_connection()

@app.post("/charge-command")
async def charge_command(request: ChargeCommandRequest):
    # 로봇 ID에 따라 다른 로직 실행
    if request.chargerId == 1:
        # 실제 충전 로직
        response = actual_charge_process(request)

    else:
        # 가짜 충전 로직
        asyncio.create_task(fake_charge_process(request))
        response = {"status": "success", "message": "Fake charge process initiated", "data": request}

    return response

@app.get("/")
async def test():
    return "ngrok 프로그램이 정상 작동하고 있쨩~"

@app.get("/return-to-start")
async def return_to_start():
    filename = "charge_command.json"

    # JSON 파일을 열어 데이터를 로드
    with open(filename, "r", encoding="utf-8") as f:
        data = json.load(f)

    # latitude와 longitude 값을 업데이트
    data['latitude'] = 340
    data['longitude'] = 122

    # 수정된 데이터를 다시 JSON 파일에 저장
    with open(filename, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False)

    return {"status": "success", "message": "return to start", "data": data}

# 서버 실행 명령어
# uvicorn charge_command:app --reload --host 0.0.0.0 --port 8002

# 가상환경 활성화 및 비활성화
# source venv/Scripts/activate

# 다른 환경에서 의존성 설치
# python -m venv venv  # 가상환경 생성
# source venv/bin/activate  # 가상환경 활성화 (Windows의 경우 venv\Scripts\activate)
# pip install -r requirements.txt  # 의존성 설치

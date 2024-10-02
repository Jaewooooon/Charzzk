from fastapi import FastAPI
from pydantic import BaseModel, Field

app = FastAPI()

class ChargeCommandRequest(BaseModel):
    charger_id: int = Field(..., alias="chargerId")
    reservation_id: int = Field(..., alias="reservationId")
    latitude: int = Field(..., alias="latitude")
    longitude: int = Field(..., alias="longitude")
    vehicle_number: str = Field(..., alias="vehicleNumber")
    charge_duration: int = Field(..., alias="chargeDuration")

# 충전 로직 함수 -> 충전로봇이 움직일 수 있게 해야함.
def actual_charge_process(request: ChargeCommandRequest):
    # charger_id가 1일 때, 실제 충전 명령 로직을 처리하는 부분
    # 추후 로직 구현 예정
    return {"status": "success", "message": "Actual charge process initiated", "data": request}

def fake_charge_process(request: ChargeCommandRequest):
    # charger_id가 2일 때, 가짜 충전 명령 로직을 처리하는 부분
    # 추후 로직 구현 예정
    return {"status": "success", "message": "Fake charge process initiated", "data": request}


@app.post("/charge-command")
async def charge_command(request: ChargeCommandRequest):
    # 로봇 ID에 따라 다른 로직 실행
    if request.charger_id == 1:
        # 실제 충전 로직
        response = actual_charge_process(request)
    elif request.charger_id == 2:
        # 가짜 충전 로직
        response = fake_charge_process(request)
    else:
        response = {"status": "error", "message": "Unknown Charger ID", "data": request}

    return response

# 서버 실행 명령어
# uvicorn main:app --reload --host 0.0.0.0 --port 8001

# 가상환경 활성화 및 비활성화
# source venv/Scripts/activate

# 다른 환경에서 의존성 설치
# python -m venv venv  # 가상환경 생성
# source venv/bin/activate  # 가상환경 활성화 (Windows의 경우 venv\Scripts\activate)
# pip install -r requirements.txt  # 의존성 설치
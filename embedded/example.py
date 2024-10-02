# 임베디드 -> 백엔드 통신하는 법

# 0. 모듈이 있는 디렉토리 경로 가져오기
import sys
import os

module_path = os.path.abspath(os.path.join('..', 'embedded'))
if module_path not in sys.path:
    sys.path.append(module_path)

# 1. MessageProducer import하기
from message_producer import MessageProducer

# 2. MessageProducer 인스턴스 생성
producer = MessageProducer()

# 3. 전송할 메시지 생성(아래 4종류의 메시지에 따라 양식 다르게...)

# 로봇의 인식 상태 업데이트
recognition_status_success_message = {
  "type": "recognition_status",
  "status": "success",
  "reservationId": 1,
  "chargerId": 1,
  "vehicleNumber": "11가1234",
  "timestamp": "2024-09-16T14:00:00Z"
}

recognition_status_failure_message= {
  "type": "recognition_status",
  "status": "failure",
  "reservationId": 1,
  "chargerId": 1,
  "vehicleNumber": "11가1234",
  "timestamp": "2024-09-16T14:00:00Z"
}

# 배터리 상태 메시지
battery_status_message = {
  "type": "battery_status",
  "batteryLevel": 85,
  "reservationId": 1,
  "chargerId": 1,
  "vehicleNumber": "11가1234",  
  "timestamp": "2024-09-16T14:05:00Z"
}

# 충전 완료 메시지
charge_complete_message = {
  "type": "charge_complete",
  "reservationId": 1,
  "chargerId": 1,
  "vehicleNumber": "11가1234", 
  "timestamp": "2024-09-16T14:30:00Z"
}

# 충전 로봇 위치 메시지
location_update_message = {
  "type": "location_update",
  "reservationId": 1,
  "chargerId": 1,
  "vehicleNumber": "11가1234",
  "latitude": 255,
  "longitude": 311, 
  "timestamp": "2024-09-16T14:30:00Z"
}


# 4. 메시지 전송
producer.send_message_to_queue(charge_complete_message)

# 5. 작업 완료 후 연결 종료
producer.close_connection()

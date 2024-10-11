import os
import time
import datetime
import json
from message_producer import MessageProducer # 뭔가 import 맞게 해야할듯...!


class Charging:
    def __init__(self, duration, batteryLevel, reservationId, chargerId, carNumber):
        self.duration = duration  # 분 단위 시간을 초인 것처럼 사용
        self.batteryLevel = batteryLevel  # 초기 배터리 레벨 설정
        self.reservationId = reservationId
        self.chargerId = chargerId
        self.carNumber = carNumber
        self.producer = MessageProducer()  # RabbitMQ MessageProducer 인스턴스 생성

    def send_battery_status(self):
        # 배터리 상태 메시지 전송
        battery_status_message = {
            "type": "battery_status",
            "batteryLevel": self.batteryLevel,
            "reservationId": self.reservationId,
            "chargerId": self.chargerId,
            "carNumber": self.carNumber,
            "timestamp": datetime.datetime.now().isoformat()  # 현재 시간으로 타임스탬프 생성
        }
        self.producer.send_message_to_queue(battery_status_message)
        print(f"Battery Status Sent: {battery_status_message}")

    def send_charge_complete(self):
        # 충전 완료 메시지 전송
        charge_complete_message = {
            "type": "charge_complete",
            "reservationId": self.reservationId,
            "chargerId": self.chargerId,
            "carNumber": self.carNumber,
            "timestamp": datetime.datetime.now().isoformat()
        }
        self.producer.send_message_to_queue(charge_complete_message)
        print(f"Charge Complete Sent: {charge_complete_message}")

    def start_charging(self):
        # 매 초마다 배터리 1% 증가시키고 메시지 전송
        for second in range(self.duration):
            if self.batteryLevel < 100:  # 배터리 레벨이 100% 이하일 경우
                self.batteryLevel += 1
                self.send_battery_status()  # 1초마다 배터리 상태 전송
                time.sleep(1)  # 실제 1초 대기 (1분을 1초로 가정한 부분)

        # 충전이 완료되면 "charge_complete" 메시지 전송
        self.send_charge_complete()

        # 모든 작업이 완료되었으므로 RabbitMQ 연결 종료
        self.producer.close_connection()

# 실행
if __name__ == "__main__":

    filename_txt="charge_command.json"
    filename = os.path.join("C:\\Users\\SSAFY\\Desktop\\ros2_ws\\src\\sub1\\sub1", filename_txt)  # 절대 경로 설정
        
    # charge_command.json 읽는 로직 로컬에 맞게 수정
    with open(filename, "r",encoding="utf-8") as f:
        charge_data = json.load(f)

    # JSON 데이터에서 필요한 값을 추출하여 Charging 클래스 인스턴스 생성
    charging = Charging(
        duration=charge_data['duration'],  # JSON에서 'duration' 값 그대로 사용
        batteryLevel=charge_data['batteryLevel'],  # JSON에서 'batteryLevel' 값 그대로 사용
        reservationId=charge_data['reservationId'],  # JSON에서 'reservationId' 값 그대로 사용
        chargerId=charge_data['chargerId'],  # JSON에서 'chargerId' 값 그대로 사용
        carNumber=charge_data['carNumber']  # JSON에서 'carNumber' 값 그대로 사용
    )
    charging.start_charging()

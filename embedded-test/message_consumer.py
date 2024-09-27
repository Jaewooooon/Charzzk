import pika
import json

# RabbitMQ 서버에 연결
connection = pika.BlockingConnection(pika.ConnectionParameters('localhost'))
channel = connection.channel()

# Spring Boot에서 전송한 메시지를 수신할 큐 선언
channel.queue_declare(queue='spring_to_python_queue', durable=True)

# 메시지 수신 콜백 함수
def callback(ch, method, properties, body):
    # 수신된 메시지를 JSON 형식으로 파싱
    message = json.loads(body.decode())
    message_type = message.get("type")
    robot_serial_number = message.get("robotSerialNumber")
    data = message.get("data")

    # 메시지 유형에 따라 처리
    if message_type == "charge_command":
        vehicle_id = data.get("vehicleId")
        charge_start_time = data.get("chargeStartTime")
        charge_duration = data.get("chargeDuration")
        charge_station_id = data.get("chargeStationId")
        print(f"Charge command received for vehicle {vehicle_id} by Robot {robot_serial_number} at {charge_start_time} for {charge_duration} minutes at station {charge_station_id}")

# 큐로부터 메시지 수신
channel.basic_consume(queue='spring_to_python_queue', on_message_callback=callback, auto_ack=True)

print('Waiting for messages from Spring Boot...')
channel.start_consuming()

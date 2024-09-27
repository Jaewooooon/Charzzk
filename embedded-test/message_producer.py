import pika
import json
from datetime import datetime

# RabbitMQ 서버에 연결
connection = pika.BlockingConnection(pika.ConnectionParameters('localhost'))
channel = connection.channel()

# 큐 선언 (Spring Boot에서 수신할 큐)
channel.queue_declare(queue='python_to_spring_queue', durable=True)
channel.queue_declare(queue='robot_location_queue', durable=True)

def send_message_to_queue(queue_name, message):
    # JSON 형식으로 메시지 직렬화
    json_message = json.dumps(message)

    # 메시지 전송
    channel.basic_publish(
        exchange='',
        routing_key=queue_name,
        body=json_message,
        properties=pika.BasicProperties(
            delivery_mode=2,  # 메시지의 지속성 보장
        )
    )
    print(f"Sent message to {queue_name}: {json_message}")

# 예시 메시지 생성 (로봇의 인식 상태 업데이트)
recognition_status_message = {
    "type": "recognition_status",
    "robotSerialNumber": "robot_001",
    "data": {
        "vehicleId": "AB4411",
        "status": "success",
        "timestamp": datetime.now().isoformat()
    }
}

# 예시 메시지 생성 (로봇의 위치 업데이트)
location_update_message = {
    "type": "location_update",
    "robotSerialNumber": "robot_001",
    "data": {
        "latitude": "37.7749",
        "longitude": "-122.4194",
        "timestamp": datetime.now().isoformat()
    }
}

# 메시지를 각 큐에 전송
# send_message_to_queue('python_to_spring_queue', recognition_status_message)
send_message_to_queue('robot_location_queue', location_update_message)

connection.close()
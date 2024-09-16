
import pika

# RabbitMQ 서버에 연결
connection = pika.BlockingConnection(pika.ConnectionParameters('localhost'))
channel = connection.channel()

# 큐 선언 (Spring Boot에서 수신할 큐)
channel.queue_declare(queue='python_to_spring_queue', durable=True)

# 메시지 전송
message = "Hello from Python to Spring Boot!"
channel.basic_publish(
    exchange='',
    routing_key='python_to_spring_queue',
    body=message,
    properties=pika.BasicProperties(
        delivery_mode=2, # 메시지의 지속성 보장
    )
)

print(f"Sent message to Spring Boot: {message}")
connection.close()
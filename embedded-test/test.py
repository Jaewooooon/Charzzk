# pika install 해야함(pip install pika 였나)

import pika

# RabbitMQ 서버에 연결 설정
connection = pika.BlockingConnection(pika.ConnectionParameters('localhost'))
channel = connection.channel()

# Spring Boot에서 사용하는 큐와 동일한 큐 선언
channel.queue_declare(queue='python_to_spring_queue', durable=True)
channel.queue_declare(queue='spring_to_python_queue', durable=True)

# 메시지 전송 (EM -> BE)
channel.basic_publish(exchange='',
                      routing_key='python_to_spring_queue',
                      body='Recognition Success')
print("Sent 'Recognition Success' message to python_to_spring_queue")

# 메시지 수신 (BE -> EM)
def callback(ch, method, properties, body):
    print(f"Received message from BE: {body.decode()}")

channel.basic_consume(queue='spring_to_python_queue', on_message_callback=callback, auto_ack=True)

print('Waiting for messages. To exit press CTRL+C')
channel.start_consuming()

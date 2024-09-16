import pika

# RabbitMQ 서버에 연결
connection = pika.BlockingConnection(pika.ConnectionParameters('localhost'))
channel = connection.channel()

# 큐 선언
channel.queue_declare(queue='my_queue', durable=True)

# 메시지 수신 콜백 함수
def callback(ch, method, properties, body):
    print("Received message: %r" % body.decode())
    # 메시지 처리 로직 추가

# 큐로부터 메시지 수신
channel.basic_consume(queue='my_queue', on_message_callback=callback, auto_ack=True)

print('Waiting for messages. To exit press CTRL+C')
channel.start_consuming()

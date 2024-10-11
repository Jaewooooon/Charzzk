import pika
import json

class MessageProducer:
    def __init__(self, host='localhost', exchange_name='exchange', routing_key='routingkey', queue_name='python_to_spring_queue'):
        self.host = host
        self.exchange_name = exchange_name
        self.routing_key = routing_key
        self.queue_name = queue_name
        self.connection = None
        self.channel = None
        self._setup_connection()

    def _setup_connection(self):
        # RabbitMQ 서버에 연결
        self.connection = pika.BlockingConnection(pika.ConnectionParameters(self.host))
        self.channel = self.connection.channel()

        # Exchange 선언
        self.channel.exchange_declare(exchange=self.exchange_name, exchange_type='direct', durable=True)

        # 큐 선언
        self.channel.queue_declare(queue=self.queue_name, durable=True)

        # 큐와 Exchange를 바인딩
        self.channel.queue_bind(exchange=self.exchange_name, queue=self.queue_name, routing_key=self.routing_key)

    def send_message_to_queue(self, message):
        json_message = json.dumps(message, ensure_ascii=False)
        self.channel.basic_publish(
            exchange=self.exchange_name,
            routing_key=self.routing_key,
            body=json_message,
            properties=pika.BasicProperties(
                delivery_mode=2,
                content_type='application/json',
                content_encoding='utf-8'
            )
        )
        print(f"Sent message: {json_message}")

    def close_connection(self):
        self.connection.close()

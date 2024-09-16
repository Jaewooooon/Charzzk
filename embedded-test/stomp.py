# pip install stomp.py

# rabbitmq-plugins enable rabbitmq_stomp => 도커쓴다면 도커 내부 쉘에서 해야함.
# 기본적으로 rabbitMQ의 STOMP 포트는 61613임. 이 포트로 python embedded서버와 spring 백엔드 서버 연결


import stomp
import time

class MyListener(stomp.ConnectionListener):
    def on_error(self, headers, message):
        print('received an error "%s"' % message)

    def on_message(self, headers, message):
        print('received a message "%s"' % message)

# STOMP 연결 설정
conn = stomp.Connection([('localhost', 61613)])  # RabbitMQ 또는 STOMP 서버의 IP와 포트
conn.set_listener('', MyListener())
conn.start()
conn.connect('guest', 'guest', wait=True)

# 1) 인식 성공 메시지 전송
conn.send(body="Recognition Success", destination='/app/recognition')

# 2) 인식 실패 메시지 전송
conn.send(body="Recognition Failed", destination='/app/recognition')

# 3) 배터리 상태 메시지 전송
conn.send(body="Battery Status: 80%", destination='/app/battery')

# 4) 충전 완료 메시지 전송
conn.send(body="Charging Complete", destination='/app/battery')

# BE → EM 명령 수신
conn.subscribe(destination='/topic/command', id=1, ack='auto')

time.sleep(5)  # 메시지 수신 대기 시간
conn.disconnect()

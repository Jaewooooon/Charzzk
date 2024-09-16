package com.ssafy.charzzk.rabbitmq;

import org.springframework.amqp.rabbit.annotation.RabbitListener;
import org.springframework.stereotype.Service;

@Service
// RabbitMQ 큐로부터 메시지 수신하여 처리하는 역할하는 클래스
public class MessageConsumer {

    // @RabbitListener: 지정된 큐에 대한 리스너 정의
    // queues = ~ : 리스너가 수신할 큐의 이름 지정
    @RabbitListener(queues = RabbitMQConfig.PYTHON_TO_SPRING_QUEUE)
    public void receiveMessage(String message) {
        System.out.println("Received message from Python: " + message);
        // 메시지 처리 로직 추가

        // 1) 번호판 인식 성공 or 실패
        // 2) 배터리 상태 표시 및 충전 완료
    }
}

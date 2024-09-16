package com.ssafy.charzzk.rabbitmq;

import org.springframework.amqp.rabbit.core.RabbitTemplate;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

@Service
public class MessageProducer {
    // Spring 애플리케이션에서 RabbitMQ를 통해 메시지를 전송하는 역할을 하는 클래스
    // 서비스 계층에서 RabbitMQ의 특정 큐로 메시지 보내게 됨

    // Spring에서 제공하는 템플릿 클래스 -> 편하게 쓰게 해주는 듯
    // private final은 불변 필드 -> 멀티스레드 환경에서 안정성 보장
    private final RabbitTemplate rabbitTemplate;

    @Autowired
    // 생성자 주입 -> 이렇게 해도 되나?
    // RabbitTemplate 객체 주입한 것
    public MessageProducer(RabbitTemplate rabbitTemplate) {
        this.rabbitTemplate = rabbitTemplate;
    }

    // 충전 시작해라! 명령 보내는 메서드
    public void sendChargeCommand(String command) {
        rabbitTemplate.convertAndSend(RabbitMQConfig.SPRING_TO_PYTHON_QUEUE, command);
        System.out.println("Sent command to EM: " + command);
    }
}

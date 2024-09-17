package com.ssafy.charzzk.rabbitmq;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import org.springframework.amqp.rabbit.core.RabbitTemplate;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

import java.time.Instant;
import java.util.HashMap;
import java.util.Map;

@Service
public class MessageProducer {
    // Spring 애플리케이션에서 RabbitMQ를 통해 메시지를 전송하는 역할을 하는 클래스
    // 서비스 계층에서 RabbitMQ의 특정 큐로 메시지 보내게 됨

    // Spring에서 제공하는 템플릿 클래스 -> 편하게 쓰게 해주는 듯
    // private final은 불변 필드 -> 멀티스레드 환경에서 안정성 보장
    private final RabbitTemplate rabbitTemplate;
    private final ObjectMapper objectMapper; // json 직렬화를 위한 ObjectMapper

    @Autowired
    // 생성자 주입 -> 이렇게 해도 되나?
    // RabbitTemplate, objectMapper 객체 주입한 것
    public MessageProducer(RabbitTemplate rabbitTemplate, ObjectMapper objectMapper) {
        this.rabbitTemplate = rabbitTemplate;
        this.objectMapper = objectMapper;
    }

    // 충전 시작해라! 명령 보내는 메서드
    // JSON 형식으로 충전 명령 메시지를 RabbitMQ 큐에 전송하는 메서드
    public void sendChargeCommand(String vehicleId, String robotSerialNumber, String chargeStartTime, int chargeDuration, String chargeStationId) {
        Map<String, Object> commandMap = new HashMap<>();
        commandMap.put("type", "charge_command");
        commandMap.put("robotSerialNumber", robotSerialNumber);
        Map<String, Object> data = new HashMap<>();
        data.put("vehicleId", vehicleId);
        data.put("chargeStartTime", chargeStartTime);
        data.put("chargeDuration", chargeDuration);
        data.put("chargeStationId", chargeStationId);
        commandMap.put("data", data);

        try {
            String jsonMessage = objectMapper.writeValueAsString(commandMap); // Map을 JSON 형식으로 변환
            rabbitTemplate.convertAndSend(RabbitMQConfig.SPRING_TO_PYTHON_QUEUE, jsonMessage);
            System.out.println("Sent JSON command to EM (Python): " + jsonMessage);
        } catch (JsonProcessingException e) {
            e.printStackTrace(); // 실제 환경에서는 로깅 프레임워크 쓰는 게 더 좋다함.
        }
    }
}

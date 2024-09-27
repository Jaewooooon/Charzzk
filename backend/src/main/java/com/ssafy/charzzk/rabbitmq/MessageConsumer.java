package com.ssafy.charzzk.rabbitmq;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import org.springframework.amqp.rabbit.annotation.RabbitListener;
import org.springframework.messaging.simp.SimpMessagingTemplate;
import org.springframework.stereotype.Service;

@Service
// RabbitMQ 큐로부터 메시지 수신하여 처리하는 역할하는 클래스
public class MessageConsumer {

    private final ObjectMapper objectMapper;
    private final SimpMessagingTemplate messagingTemplate; // 웹소켓 메시지 전송을 위한 템플릿

    // 생성자 하나라 자동주입됨
    public MessageConsumer(ObjectMapper objectMapper, SimpMessagingTemplate messagingTemplate) {
        this.objectMapper = objectMapper;
        this.messagingTemplate = messagingTemplate;
    }

    // @RabbitListener: 지정된 큐에 대한 리스너 정의
    // queues = ~ : 리스너가 수신할 큐의 이름 지정
    @RabbitListener(queues = RabbitMQConfig.PYTHON_TO_SPRING_QUEUE)
    public void receiveMessage(String message) {
        try {
            JsonNode jsonNode = objectMapper.readTree(message);

            // 필수 필드 체크
            if (!jsonNode.has("type") || !jsonNode.has("robotSerialNumber") || !jsonNode.has("data")) {
                System.out.println("Invalid message format: " + message);
                return;
            }

            String type = jsonNode.get("type").asText();
            String robotSerialNumber = jsonNode.get("robotSerialNumber").asText();
            JsonNode data = jsonNode.get("data");

            String timestamp = data.has("timestamp") ? data.get("timestamp").asText() : "N/A";

            switch (type) {
                case "recognition_status":
                    handleRecognitionStatus(data, robotSerialNumber, timestamp);
                    break;
                case "battery_status":
                    handleBatteryStatus(data, robotSerialNumber, timestamp);
                    break;
                case "charge_complete":
                    handleChargeComplete(data, robotSerialNumber, timestamp);
                    break;
                default:
                    System.out.println("Unknown message type: " + type);
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    // robot_location_queue에서 로봇 위치 정보 수신
    @RabbitListener(queues = RabbitMQConfig.ROBOT_LOCATION_QUEUE)
    public void receiveLocationMessage(String message) {
        try {
            JsonNode jsonNode = objectMapper.readTree(message);

            // 필수 필드 체크
            if (!jsonNode.has("robotSerialNumber") || !jsonNode.has("data")) {
                System.out.println("Invalid location message format: " + message);
                return;
            }

            String robotSerialNumber = jsonNode.get("robotSerialNumber").asText();
            JsonNode data = jsonNode.get("data");

            // webSocket 을 통해 위치 정보를 프론트엔드로 전송
            messagingTemplate.convertAndSend("/topic/location", data);
            System.out.println("Sent location update to frontend via WebSocket: " + data);

            // 나중에 DB 저장하는 로직 추가할 수도.
            // handleLocationData(data, robotSerialNumber);

        } catch (Exception e) {
            e.printStackTrace();
        }
    }


    private void handleRecognitionStatus(JsonNode data, String robotSerialNumber, String timestamp) {
        String vehicleId = data.get("vehicleId").asText();
        String status = data.get("status").asText();
        System.out.println("Vehicle " + vehicleId + " recognition status: " + status + " by Robot " + robotSerialNumber + " at " + timestamp);
    }

    private void handleBatteryStatus(JsonNode data, String robotSerialNumber, String timestamp) {
        String vehicleId = data.get("vehicleId").asText();
        int batteryLevel = data.get("batteryLevel").asInt();
        System.out.println("Vehicle " + vehicleId + " battery level: " + batteryLevel + "%, reported by Robot " + robotSerialNumber + " at " + timestamp);
    }

    private void handleChargeComplete(JsonNode data, String robotSerialNumber, String timestamp) {
        String vehicleId = data.get("vehicleId").asText();
        System.out.println("Vehicle " + vehicleId + " charge completed by Robot " + robotSerialNumber + " at " + timestamp);
    }
}

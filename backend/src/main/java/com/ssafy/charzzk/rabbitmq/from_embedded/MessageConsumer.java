package com.ssafy.charzzk.rabbitmq.from_embedded;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.ssafy.charzzk.api.service.message.MessageService;
import lombok.RequiredArgsConstructor;
import org.springframework.amqp.rabbit.annotation.RabbitListener;
import org.springframework.stereotype.Service;

@RequiredArgsConstructor
@Service
public class MessageConsumer {

    private final MessageService messageService;

    @RabbitListener(queues = "${spring.rabbitmq.queue.name}")
    public void receiveMessage(MessageRequest  message) {
        try {
            // type에 따라 다른 로직 실행
            switch (message.getType()) {
                case "recognition_status":
                    handleRecognitionStatus(message);
                    break;
                case "battery_status":
                    handleBatteryStatus(message);
                    break;
                case "charge_complete":
                    handleChargeComplete(message);
                    break;
                case "location_update":
                    handleLocationUpdate(message);
                    break;
                default:
                    System.out.println("Unknown message type: " + message.getType());
            }

        } catch (Exception e) {
            System.err.println("Failed to process message: " + e.getMessage());
        }
    }

    private void handleRecognitionStatus(MessageRequest message) {
        System.out.println("Handling recognition status: " + message);
        // 인식 성공/실패 처리 로직 추가
    }

    private void handleBatteryStatus(MessageRequest message) {
        System.out.println("Handling battery status: " + message);

        messageService.updateBatteryStatus(message.getReservationId(), message.getBatteryLevel());
    }

    private void handleChargeComplete(MessageRequest message) {
        System.out.println("Handling charge complete: " + message);

        messageService.chargeComplete(message.getReservationId());
    }

    private void handleLocationUpdate(MessageRequest message) {
        System.out.println("Handling location update: " + message);
        // 위치 업데이트 처리 로직 추가
    }
}

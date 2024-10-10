package com.ssafy.charzzk.api.service.message;

import com.ssafy.charzzk.api.service.message.request.MessageRequest;
import lombok.RequiredArgsConstructor;
import org.springframework.amqp.rabbit.annotation.RabbitListener;
import org.springframework.stereotype.Service;

@RequiredArgsConstructor
@Service
public class MessageConsumer {

    private final MessageService messageService;

    @RabbitListener(queues = "${spring.rabbitmq.queue.name}")
    public void receiveMessage(MessageRequest message) {
        try {
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
    }
}

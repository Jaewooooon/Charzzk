package com.ssafy.charzzk.rabbitmq;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.web.bind.annotation.*;

@RestController
public class TestController {

    private final MessageProducer messageProducer;

    @Autowired
    // MessageProducer 빈 생성자 주입
    public TestController(MessageProducer messageProducer) {
        this.messageProducer = messageProducer;
    }

//    @GetMapping("/send/{message}")
//    public String sendMessage(@PathVariable String message) {
//        //sendChargeCommand 에 message 인자로 넣어 RabbitMQ 큐로 전송
//        messageProducer.sendChargeCommand(message);
//        return "Message sent: " + message;
//    }


    @PostMapping("/send/charge-command")
    public String sendChargeCommand(@RequestBody ChargeCommandRequest request) {

        // JSON 형식으로 메시지를 전송
        messageProducer.sendChargeCommand(
                request.getVehicleId(),
                request.getRobotSerialNumber(),
                request.getChargeStartTime(),
                request.getChargeDuration(),
                request.getChargeStationId()
        );

        return "Charge command sent for vehicle: " + request.getVehicleId() + " by robot: " + request.getRobotSerialNumber();
    }
}

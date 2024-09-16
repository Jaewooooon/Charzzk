package com.ssafy.charzzk.rabbitmq;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PathVariable;
import org.springframework.web.bind.annotation.RequestParam;
import org.springframework.web.bind.annotation.RestController;

@RestController
public class TestController {

    private final MessageProducer messageProducer;

    @Autowired
    // MessageProducer 빈 생성자 주입
    public TestController(MessageProducer messageProducer) {
        this.messageProducer = messageProducer;
    }

    @GetMapping("/send/{message}")
    public String sendMessage(@PathVariable String message) {
        //sendChargeCommand 에 message 인자로 넣어 RabbitMQ 큐로 전송
        messageProducer.sendChargeCommand(message);
        return "Message sent: " + message;
    }
}

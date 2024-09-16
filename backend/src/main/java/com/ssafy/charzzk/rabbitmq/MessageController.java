package com.ssafy.charzzk.rabbitmq;

import org.springframework.messaging.handler.annotation.MessageMapping;
import org.springframework.messaging.handler.annotation.SendTo;
import org.springframework.stereotype.Controller;

@Controller
// Spring 어플리케이션에서 WebSocket 을 통해 메시지 처리하고 클라이언트에게 응답 전송하는 클래스
public class MessageController {

    // 클라이언트가 /app/recognition 로 전송한 메시지 처리 메서드
    @MessageMapping("/recognition")
    // 처리의 결과를 /topic/recognition 주제를 구독하는 모든 클라이언트에게 전송(STOMP의 Pub-Sub구조..)
    @SendTo("/topic/recognition")
    // 메시지를 받아 처리하고, 결과 문자열 반환
    public String handleRecognitionMessage(String message) {
        System.out.println("Received recognition message: " + message);
        return "Recognition processed: " + message;
    }

    // 클라이언트가 /app/battery 경로로 전송한 메시지 처리
    @MessageMapping("/battery")
    // 처리 결과를 /topic/battery 주제 구독하는 클라이언트에게 전송
    @SendTo("/topic/battery")
    public String handleBatteryMessage(String message) {
        System.out.println("Received battery status message: " + message);
        return "Battery status processed: " + message;
    }

    @MessageMapping("/sendCommand")
    @SendTo("/topic/command")
    public String sendChargeCommand(String command) {
        System.out.println("Sending charge command to EM: " + command);
        return "Command sent to EM: " + command;
    }
}

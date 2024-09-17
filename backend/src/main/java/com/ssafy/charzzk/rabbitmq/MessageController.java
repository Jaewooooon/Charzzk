package com.ssafy.charzzk.rabbitmq;

import org.springframework.messaging.handler.annotation.MessageMapping;
import org.springframework.messaging.handler.annotation.SendTo;
import org.springframework.stereotype.Controller;

@Controller
// Spring 어플리케이션에서 WebSocket 을 통해 메시지 처리하고 클라이언트에게 응답 전송하는 클래스
public class MessageController {

    // 클라이언트가 /app/sendMessage 경로로 메시지 전송하면 호출되는 메서드
    @MessageMapping("/sendMessage")
    // 처리의 결과를 /topic/messages 주제를 구독하는 모든 클라이언트에게 전송(STOMP의 Pub-Sub구조..)
    @SendTo("/topic/messages")
    // 메시지를 받아 처리하고, 결과 문자열 반환
    public String broadcastMessage(String message) {
        // 받은 메시지 가공하거나 처리할 수 있음
        System.out.println("Received recognition message: " + message);
        return "message: " + message;
    }

}

package com.ssafy.charzzk.rabbitmq;

import org.springframework.context.annotation.Configuration;
import org.springframework.messaging.simp.config.MessageBrokerRegistry;
import org.springframework.web.socket.config.annotation.EnableWebSocketMessageBroker;
import org.springframework.web.socket.config.annotation.StompEndpointRegistry;
import org.springframework.web.socket.config.annotation.WebSocketMessageBrokerConfigurer;

@Configuration
@EnableWebSocketMessageBroker //WebSocket 메시지 브로커를 활성화하겠다
public class WebSocketConfig implements WebSocketMessageBrokerConfigurer {

    @Override
    // websocket 메시지 브로커 설정 구성 메서드
    public void configureMessageBroker(MessageBrokerRegistry config) {

        // in-memory 메시지 브로커를 활성화하고, 메시지를 브로드캐스트할 경로 지정
        // topic 은 보통 다중 구독자 모델에 사용됨(여러 클라이언트가 특정 주제(topic)에 대한 메시지 수신
        config.enableSimpleBroker("/topic");

        // 클라이언트가 서버로 메시지를 보낼 때 사용하는 경로의 접두사 지정
        config.setApplicationDestinationPrefixes("/app");
    }

    @Override
    // 클아이언트가 WebSocket 서버에 연결할 수 있는 STOMP 엔드포인트 등록 메서드
    public void registerStompEndpoints(StompEndpointRegistry registry) {
        // 클라이언트가 "/stomp-websocket 엔드 포인트를 통해 Websocket 에 연결할 수 있도록 설정
        // CORS 설정으로 모든 도메인에서의 요청 허용 -> 실제 운영에서는 보안 때문에 특정 도메인으로 하는 게 좋다함
        // withSockJS. SockJS 지원을 추가하는 것. 브라우저에서 폴백 옵션으로 사용할 수 있는 라이브러리-> WebSocket 지원하지 않는 환경에서도 웹소켓과 유사한 기능 사용 가능
        registry.addEndpoint("/ws").setAllowedOrigins("*").withSockJS();
    }
}

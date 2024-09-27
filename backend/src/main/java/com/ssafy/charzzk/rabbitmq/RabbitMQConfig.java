package com.ssafy.charzzk.rabbitmq;

import org.springframework.amqp.core.Queue;
import org.springframework.amqp.rabbit.connection.CachingConnectionFactory;
import org.springframework.amqp.rabbit.core.RabbitTemplate;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;

@Configuration
public class RabbitMQConfig {
    // Spring 어플리케이션이 RabbitMQ 메시지 브로커와 통신하기 위한 기본 설정을 정의하는 클래스임

    public static final String PYTHON_TO_SPRING_QUEUE = "python_to_spring_queue"; // EM -> BE 일반 큐 이름
    public static final String SPRING_TO_PYTHON_QUEUE = "spring_to_python_queue"; // BE -> EM 일반 큐 이름
    public static final String ROBOT_LOCATION_QUEUE = "robot_location_queue";     // EM -> BE 로봇 위치 큐 이름

    @Bean
    public CachingConnectionFactory connectionFactory() {
        CachingConnectionFactory connectionFactory = new CachingConnectionFactory("localhost");
        connectionFactory.setUsername("guest");
        connectionFactory.setPassword("guest");
        return connectionFactory;
    }

    // 큐 생성
    @Bean
    public Queue pythonToSpringQueue() {
        return new Queue(PYTHON_TO_SPRING_QUEUE, true); // 영구 큐 생성
    }

    @Bean
    public Queue springToPythonQueue() {
        return new Queue(SPRING_TO_PYTHON_QUEUE, true); // 영구 큐 생성
    }

    @Bean
    public Queue robotLocationQueue() {
        return new Queue(ROBOT_LOCATION_QUEUE, true); // 로봇 위치 정보 큐 추가
    }

    // RabbitTemplate 은 Spring 에서 RabbitMQ 와의 통신을 위해 제공하는 템플릿 클래스임
    @Bean
    public RabbitTemplate rabbitTemplate(CachingConnectionFactory connectionFactory) {
        return new RabbitTemplate(connectionFactory);
    }
}

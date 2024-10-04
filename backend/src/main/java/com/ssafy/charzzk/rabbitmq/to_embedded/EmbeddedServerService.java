package com.ssafy.charzzk.rabbitmq.to_embedded;


import org.springframework.stereotype.Service;
import org.springframework.web.client.RestTemplate;

@Service
public class EmbeddedServerService {
    private final RestTemplate restTemplate;

    public EmbeddedServerService(RestTemplate restTemplate) {
        this.restTemplate = restTemplate;
    }

    public String sendChargeCommand(ChargeCommandRequest request) {
        String url = "http://localhost:8001/charge-command"; // FastAPI 서버의 URL

        // 응답 처리
        return restTemplate.postForObject(url, request, String.class);
    }
}

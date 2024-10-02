package com.ssafy.charzzk.rabbitmq.to_embedded;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RestController;

@RestController
public class ChargeController {
    private final EmbeddedServerService embeddedServerService;

    @Autowired
    public ChargeController(EmbeddedServerService embeddedServerService) {
        this.embeddedServerService = embeddedServerService;
    }

    @PostMapping("/start-charge")
    public String startCharge() {

        // 테스트 용
        ChargeCommandRequest request = new ChargeCommandRequest();
        request.setReservationId(1L);
        request.setChargerId(2L);
        request.setLatitude(167);
        request.setLongitude(257);
        request.setVehicleNumber("EV12345");
        request.setChargeDuration(120);

        return embeddedServerService.sendChargeCommand(request);
    }
}

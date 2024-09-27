package com.ssafy.charzzk.rabbitmq;

import lombok.Getter;
import lombok.Setter;

// 요청 데이터 받을 DTO
@Getter
@Setter
public class ChargeCommandRequest {
    private String vehicleId;
    private String robotSerialNumber;
    private String chargeStartTime;
    private int chargeDuration;
    private String chargeStationId;
}

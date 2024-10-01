package com.ssafy.charzzk.rabbitmq.to_embedded;

import lombok.Getter;
import lombok.Setter;

// 요청 데이터 받을 DTO
@Getter
@Setter
public class ChargeCommandRequest {
    private long robotId;
    private long reservationId;
    private long parkingLotId;
    private int gridX;
    private int gridY;
    private String vehicleNumber;
    private String chargeStartTime;
    private int chargeDuration; // 아마 분단위
}

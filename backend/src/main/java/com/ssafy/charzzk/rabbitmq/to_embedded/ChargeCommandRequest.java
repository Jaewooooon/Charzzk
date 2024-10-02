package com.ssafy.charzzk.rabbitmq.to_embedded;

import lombok.Getter;
import lombok.Setter;

@Getter
@Setter
public class ChargeCommandRequest {
    private long chargerId;
    private long reservationId;
    private double latitude;
    private double longitude;
    private String vehicleNumber;
    private int chargeDuration; // 분 단위
}

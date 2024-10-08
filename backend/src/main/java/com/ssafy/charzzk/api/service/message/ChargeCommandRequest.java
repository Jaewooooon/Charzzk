package com.ssafy.charzzk.api.service.message;

import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;

@Getter
@NoArgsConstructor
public class ChargeCommandRequest {
    private long chargerId;
    private long reservationId;
    private double latitude;
    private double longitude;
    private String vehicleNumber;
    private int chargeDuration;

    public ChargeCommandRequest(long chargerId, long reservationId, double latitude, double longitude, String vehicleNumber, int chargeDuration) {
        this.chargerId = chargerId;
        this.reservationId = reservationId;
        this.latitude = latitude;
        this.longitude = longitude;
        this.vehicleNumber = vehicleNumber;
        this.chargeDuration = chargeDuration;
    }
}

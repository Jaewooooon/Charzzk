package com.ssafy.charzzk.api.service.message.request;

import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Getter
@NoArgsConstructor
public class MessageRequest {
    private String type;
    private String status;  // recognition_status 에서만 사용됨
    private Long reservationId;
    private Long chargerId;
    private String vehicleNumber;
    private String timestamp;
    private Integer batteryLevel;  // battery_status 에서만 사용됨
    private Double latitude;  // location_update 에서만 사용됨
    private Double longitude;  // location_update 에서만 사용됨

    @Builder
    private MessageRequest(String type, String status, Long reservationId, Long chargerId, String vehicleNumber, String timestamp, Integer batteryLevel, Double latitude, Double longitude) {
        this.type = type;
        this.status = status;
        this.reservationId = reservationId;
        this.chargerId = chargerId;
        this.vehicleNumber = vehicleNumber;
        this.timestamp = timestamp;
        this.batteryLevel = batteryLevel;
        this.latitude = latitude;
        this.longitude = longitude;
    }
}

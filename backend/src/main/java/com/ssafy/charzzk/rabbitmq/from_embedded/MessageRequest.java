package com.ssafy.charzzk.rabbitmq.from_embedded;

import lombok.Getter;
import lombok.Setter;

@Getter
@Setter
public class MessageRequest {
    private String type;
    private String status;  // recognition_status 에서만 사용됨
    private int reservationId;
    private int chargerId;
    private String vehicleNumber;
    private String timestamp;
    private Integer batteryLevel;  // battery_status 에서만 사용됨
    private Double latitude;  // location_update 에서만 사용됨
    private Double longitude;  // location_update 에서만 사용됨
}

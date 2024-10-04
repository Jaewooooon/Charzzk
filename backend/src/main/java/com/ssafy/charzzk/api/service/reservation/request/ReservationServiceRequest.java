package com.ssafy.charzzk.api.service.reservation.request;

import lombok.Builder;
import lombok.Getter;

@Getter
public class ReservationServiceRequest {

    private Long parkingSpotId;
    private Long carId;
    private Long parkingLotId;
    private boolean fullCharge;
    private int time;

    @Builder
    private ReservationServiceRequest(Long parkingSpotId, Long carId, Long parkingLotId, boolean fullCharge, int time) {
        this.parkingSpotId = parkingSpotId;
        this.carId = carId;
        this.parkingLotId = parkingLotId;
        this.fullCharge = fullCharge;
        this.time = time;
    }

}

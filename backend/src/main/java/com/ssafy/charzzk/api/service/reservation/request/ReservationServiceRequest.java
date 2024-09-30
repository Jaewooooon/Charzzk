package com.ssafy.charzzk.api.service.reservation.request;

import lombok.Builder;
import lombok.Getter;

@Getter
public class ReservationServiceRequest {

    private Long carId;
    private Long parkingLotId;
    private boolean fullCharge;
    private int time;

    @Builder
    private ReservationServiceRequest(Long carId, Long parkingLotId, boolean fullCharge, int time) {
        this.carId = carId;
        this.parkingLotId = parkingLotId;
        this.fullCharge = fullCharge;
        this.time = time;
    }

}

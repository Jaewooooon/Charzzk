package com.ssafy.charzzk.api.service.reservation.request;


import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Getter
@NoArgsConstructor
public class ReservationCheckTimeServiceRequest {

    private Long parkingLotId;

    @Builder
    private ReservationCheckTimeServiceRequest(Long parkingLotId) {
        this.parkingLotId = parkingLotId;
    }
}

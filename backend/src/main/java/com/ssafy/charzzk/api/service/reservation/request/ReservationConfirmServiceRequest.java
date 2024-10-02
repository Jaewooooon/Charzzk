package com.ssafy.charzzk.api.service.reservation.request;

import lombok.Builder;
import lombok.Getter;

@Getter
public class ReservationConfirmServiceRequest {

    private Long reservationId;

    @Builder
    private ReservationConfirmServiceRequest(Long reservationId) {
        this.reservationId = reservationId;
    }
}

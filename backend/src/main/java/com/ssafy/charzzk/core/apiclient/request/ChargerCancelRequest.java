package com.ssafy.charzzk.core.apiclient.request;

import com.ssafy.charzzk.domain.reservation.Reservation;
import lombok.Builder;
import lombok.Getter;

@Getter
public class ChargerCancelRequest {

    private Long reservationId;
    private Long chargerId;


    @Builder
    private ChargerCancelRequest(Long reservationId, Long chargerId) {
        this.reservationId = reservationId;
        this.chargerId = chargerId;
    }


    public static ChargerCancelRequest of(Reservation reservation) {

        return ChargerCancelRequest.builder()
                .reservationId(reservation.getId())
                .chargerId(reservation.getCharger().getId())
                .build();
    }
}

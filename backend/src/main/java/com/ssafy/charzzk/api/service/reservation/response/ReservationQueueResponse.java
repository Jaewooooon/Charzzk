package com.ssafy.charzzk.api.service.reservation.response;

import com.ssafy.charzzk.domain.reservation.Reservation;
import lombok.Builder;

import java.util.List;
import java.util.Map;
import java.util.Queue;

public class ReservationQueueResponse {
    private Long chargerId;
    private List<ReservationResponse> reservations;

    @Builder
    public ReservationQueueResponse(Long chargerId, List<ReservationResponse> reservations) {
        this.chargerId = chargerId;
        this.reservations = reservations;
    }

    public static ReservationQueueResponse of(Long key, List<ReservationResponse> collect) {

        return ReservationQueueResponse.builder()
                .chargerId(key)
                .reservations(collect)
                .build();
    }
}

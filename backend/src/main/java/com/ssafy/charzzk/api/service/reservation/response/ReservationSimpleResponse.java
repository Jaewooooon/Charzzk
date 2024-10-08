package com.ssafy.charzzk.api.service.reservation.response;

import com.fasterxml.jackson.annotation.JsonFormat;
import com.ssafy.charzzk.domain.reservation.Reservation;
import lombok.Builder;
import lombok.Getter;

import java.time.LocalDateTime;

@Getter
public class ReservationSimpleResponse {

    private Long id;
    private String carNumber;
    @JsonFormat(shape = JsonFormat.Shape.STRING, pattern = "yyyy-MM-dd'T'HH:mm:ss")
    private LocalDateTime startTime;
    @JsonFormat(shape = JsonFormat.Shape.STRING, pattern = "yyyy-MM-dd'T'HH:mm:ss")
    private LocalDateTime endTime;

    @Builder
    private ReservationSimpleResponse(Long id, String carNumber, LocalDateTime startTime, LocalDateTime endTime) {
        this.id = id;
        this.carNumber = carNumber;
        this.startTime = startTime;
        this.endTime = endTime;
    }

    public static ReservationSimpleResponse from(Reservation reservation) {

        return ReservationSimpleResponse.builder()
                .id(reservation.getId())
                .carNumber(reservation.getCar().getNumber())
                .startTime(reservation.getStartTime())
                .endTime(reservation.getEndTime())
                .build();
    }
}

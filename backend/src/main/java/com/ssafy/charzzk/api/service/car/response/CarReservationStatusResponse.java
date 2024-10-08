package com.ssafy.charzzk.api.service.car.response;

import com.fasterxml.jackson.annotation.JsonFormat;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.ssafy.charzzk.domain.reservation.Reservation;
import lombok.Builder;
import lombok.Getter;

import java.time.LocalDateTime;

@Getter
public class CarReservationStatusResponse {

    private Long reservationId;
    private int battery;
    private String status;
    @JsonFormat(shape = JsonFormat.Shape.STRING, pattern = "yyyy-MM-dd'T'HH:mm:ss")
    private LocalDateTime startTime;
    @JsonFormat(shape = JsonFormat.Shape.STRING, pattern = "yyyy-MM-dd'T'HH:mm:ss")
    private LocalDateTime endTime;

    @Builder
    private CarReservationStatusResponse(Long reservationId, int battery, String status, LocalDateTime startTime, LocalDateTime endTime) {
        this.reservationId = reservationId;
        this.battery = battery;
        this.status = status;
        this.startTime = startTime;
        this.endTime = endTime;
    }

    public static CarReservationStatusResponse from(Reservation reservation) {

        return CarReservationStatusResponse.builder()
                .reservationId(reservation.getId())
                .battery(reservation.getCar().getBattery())
                .status(reservation.getStatus().name())
                .startTime(reservation.getStartTime())
                .endTime(reservation.getEndTime())
                .build();
    }

}

package com.ssafy.charzzk.api.service.reservation.response;

import com.fasterxml.jackson.annotation.JsonFormat;
import com.ssafy.charzzk.api.service.car.response.CarResponse;
import com.ssafy.charzzk.domain.reservation.Reservation;
import lombok.Builder;
import lombok.Getter;

import java.time.LocalDateTime;

@Getter
public class ReservationResponse {

    private Long id;
    private CarResponse car;
    @JsonFormat(shape = JsonFormat.Shape.STRING, pattern = "yyyy-MM-dd'T'HH:mm:ss")
    private LocalDateTime startTime;
    @JsonFormat(shape = JsonFormat.Shape.STRING, pattern = "yyyy-MM-dd'T'HH:mm:ss")
    private LocalDateTime endTime;
    private String status;

    @Builder
    private ReservationResponse(Long id, CarResponse car, LocalDateTime startTime, LocalDateTime endTime, String status) {
        this.id = id;
        this.car = car;
        this.startTime = startTime;
        this.endTime = endTime;
        this.status = status;
    }

    public static ReservationResponse from(Reservation findReservation) {

        return ReservationResponse.builder()
                .id(findReservation.getId())
                .car(CarResponse.from(findReservation.getCar()))
                .startTime(findReservation.getStartTime())
                .endTime(findReservation.getEndTime())
                .status(findReservation.getStatus().name())
                .build();
    }
}

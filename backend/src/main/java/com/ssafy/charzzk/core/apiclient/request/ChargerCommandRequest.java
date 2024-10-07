package com.ssafy.charzzk.core.apiclient.request;

import com.ssafy.charzzk.domain.reservation.Reservation;
import lombok.Builder;
import lombok.Getter;

import java.time.Duration;
import java.time.LocalDateTime;

@Getter
public class ChargerCommandRequest {
    private Long reservationId;
    private Long chargerId;
    private String carNumber;
    private int battery;
    private long duration;
    private double latitude;
    private double longitude;

    @Builder
    private ChargerCommandRequest(Long reservationId, Long chargerId, int battery, double latitude, double longitude, String carNumber, long duration) {
        this.reservationId = reservationId;
        this.chargerId = chargerId;
        this.battery = battery;
        this.latitude = latitude;
        this.longitude = longitude;
        this.carNumber = carNumber;
        this.duration = duration;
    }

    public static ChargerCommandRequest of(Reservation reservation) {
        long duration = Duration.between(reservation.getStartTime(), reservation.getEndTime()).toMinutes();

        return ChargerCommandRequest.builder()
                .reservationId(reservation.getId())
                .chargerId(reservation.getCharger().getId())
                .carNumber(reservation.getCar().getNumber())
                .battery(reservation.getCar().getBattery())
                .duration(duration)
                .latitude(reservation.getParkingSpot().getLocation().getLatitude())
                .longitude(reservation.getParkingSpot().getLocation().getLongitude())
                .build();
    }
}

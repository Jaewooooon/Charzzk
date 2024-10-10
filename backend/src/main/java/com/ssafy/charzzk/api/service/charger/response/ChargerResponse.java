package com.ssafy.charzzk.api.service.charger.response;

import com.ssafy.charzzk.api.service.parkinglot.response.ParkingLotResponse;
import com.ssafy.charzzk.api.service.reservation.response.ReservationResponse;
import com.ssafy.charzzk.api.service.reservation.response.ReservationSimpleResponse;
import com.ssafy.charzzk.domain.charger.Charger;
import com.ssafy.charzzk.domain.charger.ChargerStatus;
import com.ssafy.charzzk.domain.reservation.Reservation;
import lombok.Builder;
import lombok.Getter;

import java.util.List;

@Getter
public class ChargerResponse {

    private Long chargerId;
    private String serialNumber;
    private Integer battery;
    private ChargerStatus status;
    private List<ReservationSimpleResponse> reservations;

    @Builder
    private ChargerResponse(Long id, String serialNumber, Integer battery, ChargerStatus status, List<ReservationSimpleResponse> reservations) {
        this.chargerId = id;
        this.serialNumber = serialNumber;
        this.battery = battery;
        this.status = status;
        this.reservations = reservations;
    }

    public static ChargerResponse from(Charger charger) {

        return ChargerResponse.builder()
                .id(charger.getId())
                .serialNumber(charger.getSerialNumber())
                .battery(charger.getBattery())
                .status(charger.getStatus())
                .build();
    }

    public static ChargerResponse of(Charger charger, List<Reservation> reservations) {

        return ChargerResponse.builder()
                .id(charger.getId())
                .serialNumber(charger.getSerialNumber())
                .battery(charger.getBattery())
                .status(charger.getStatus())
                .reservations(
                        reservations.stream()
                                .map(ReservationSimpleResponse::from)
                                .toList())
                .build();
    }

}

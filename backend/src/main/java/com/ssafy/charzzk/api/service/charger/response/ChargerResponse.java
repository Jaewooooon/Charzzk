package com.ssafy.charzzk.api.service.charger.response;

import com.ssafy.charzzk.api.service.parkinglot.response.ParkingLotResponse;
import com.ssafy.charzzk.domain.charger.Charger;
import com.ssafy.charzzk.domain.charger.ChargerStatus;
import com.ssafy.charzzk.domain.parkinglot.ParkingLot;
import jakarta.persistence.*;
import lombok.Builder;
import lombok.Getter;

@Getter
public class ChargerResponse {

    private Long id;
    private String serialNumber;
    private Integer battery;
    private ChargerStatus status;

    @Builder
    private ChargerResponse(Long id, ParkingLotResponse parkingLot, String serialNumber, Integer battery, ChargerStatus status) {
        this.id = id;
        this.serialNumber = serialNumber;
        this.battery = battery;
        this.status = status;
    }

    public static ChargerResponse from(Charger charger) {
        return ChargerResponse.builder()
                .id(charger.getId())
                .serialNumber(charger.getSerialNumber())
                .battery(charger.getBattery())
                .status(charger.getStatus())
                .build();
    }
}

package com.ssafy.charzzk.api.service.charginglog.response;

import com.fasterxml.jackson.annotation.JsonFormat;
import com.ssafy.charzzk.api.service.car.response.CarChargingLogResponse;
import com.ssafy.charzzk.api.service.car.response.CarResponse;
import com.ssafy.charzzk.domain.car.Car;
import com.ssafy.charzzk.domain.charger.Charger;
import com.ssafy.charzzk.domain.charginglog.ChargingLog;
import jakarta.persistence.*;
import lombok.Builder;
import lombok.Getter;

import java.time.LocalDateTime;

@Getter
public class ChargingLogResponse {


    private Long id;
    private CarChargingLogResponse car;

    @JsonFormat(shape = JsonFormat.Shape.STRING, pattern = "yyyy-MM-dd'T'HH:mm:ss")
    private LocalDateTime startTime;
    @JsonFormat(shape = JsonFormat.Shape.STRING, pattern = "yyyy-MM-dd'T'HH:mm:ss")
    private LocalDateTime endTime;

    private Double chargeAmount;  // 추가
    private Long chargeCost;  // 요금 비율로 계산

    @Builder
    public ChargingLogResponse(Long id, CarChargingLogResponse car, LocalDateTime startTime, LocalDateTime endTime, Double chargeAmount, Long chargeCost) {
        this.id = id;
        this.car = car;
        this.startTime = startTime;
        this.endTime = endTime;
        this.chargeAmount = chargeAmount;
        this.chargeCost = chargeCost;
    }

    public static ChargingLogResponse from(ChargingLog chargingLog) {
        return ChargingLogResponse.builder()
                .id(chargingLog.getId())
                .car(CarChargingLogResponse.from(chargingLog.getCar()))
                .startTime(chargingLog.getStartTime())
                .endTime(chargingLog.getEndTime())
                .build();
    }

    public static ChargingLogResponse of(ChargingLog chargingLog, Double chargeAmount, Long chargeCost) {
        return ChargingLogResponse.builder()
                .id(chargingLog.getId())
                .car(CarChargingLogResponse.from(chargingLog.getCar()))
                .startTime(chargingLog.getStartTime())
                .endTime(chargingLog.getEndTime())
                .chargeAmount(chargeAmount)
                .chargeCost(chargeCost)
                .build();
    }

}

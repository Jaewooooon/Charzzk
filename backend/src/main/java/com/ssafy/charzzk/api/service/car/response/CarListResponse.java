package com.ssafy.charzzk.api.service.car.response;

import com.ssafy.charzzk.domain.car.Car;
import lombok.Builder;
import lombok.Getter;

@Getter
public class CarListResponse {

    private Long id;
    private CarTypeResponse carType;
    private String number;
    private String nickname;
    private boolean isCharging;
    private Long ChargeCost;
    private Double ChargeAmount;

    @Builder
    private CarListResponse(Long id, CarTypeResponse carType, String number, String nickname, boolean isCharging, Long chargeCost, Double chargeAmount) {
        this.id = id;
        this.carType = carType;
        this.number = number;
        this.nickname = nickname;
        this.isCharging = isCharging;
        this.ChargeCost = chargeCost;
        this.ChargeAmount = chargeAmount;
    }

    public static CarListResponse from(Car car) {
        return CarListResponse.builder()
                .id(car.getId())
                .carType(CarTypeResponse.from(car.getCarType()))
                .number(car.getNumber())
                .nickname(car.getNickname())
                .isCharging(car.isCharging())
                .build();
    }

    public static CarListResponse of(Car car, Double chargeAmount, Long chargeCost) {
        return CarListResponse.builder()
                .id(car.getId())
                .carType(CarTypeResponse.from(car.getCarType()))
                .number(car.getNumber())
                .nickname(car.getNickname())
                .isCharging(car.isCharging())
                .chargeAmount(chargeAmount)
                .chargeCost(chargeCost)
                .build();
    }
}

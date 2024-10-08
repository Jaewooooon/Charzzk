package com.ssafy.charzzk.api.service.car.response;

import com.ssafy.charzzk.domain.car.Car;
import com.ssafy.charzzk.domain.car.CarType;
import lombok.Builder;
import lombok.Getter;

@Getter
public class CarChargingLogResponse {
    private Long id;
    private CarTypeChargingLogResponse carType;
    private String number;
    private String nickname;

    @Builder
    public CarChargingLogResponse(Long id, CarTypeChargingLogResponse carType, String number, String nickname) {
        this.id = id;
        this.carType = carType;
        this.number = number;
        this.nickname = nickname;
    }

    public static CarChargingLogResponse from(Car car) {
        return CarChargingLogResponse.builder()
                .id(car.getId())
                .carType(CarTypeChargingLogResponse.from(car.getCarType()))
                .number(car.getNumber())
                .nickname(car.getNickname())
                .build();
    }

}

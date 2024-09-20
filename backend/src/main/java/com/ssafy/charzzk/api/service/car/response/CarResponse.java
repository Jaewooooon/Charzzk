package com.ssafy.charzzk.api.service.car.response;

import com.ssafy.charzzk.domain.car.Car;
import com.ssafy.charzzk.domain.car.CarType;
import lombok.Builder;
import lombok.Getter;

@Getter
public class CarResponse {

    private Long id;
    private CarTypeResponse carType;
    private String number;
    private String nickname;

    @Builder
    private CarResponse(Long id, CarType carType, String number, String nickname) {
        this.id = id;
        this.carType = CarTypeResponse.from(carType);
        this.number = number;
        this.nickname = nickname;
    }

    public static CarResponse from(Car car) {
        return CarResponse.builder()
                .id(car.getId())
                .carType(car.getCarType())
                .number(car.getNumber())
                .build();
    }
}

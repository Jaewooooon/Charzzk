package com.ssafy.charzzk.api.service.car.response;

import com.ssafy.charzzk.domain.car.CarType;
import lombok.Builder;
import lombok.Getter;

@Getter
public class CarTypeChargingLogResponse {

    private Long id;
    private String name;

    @Builder
    private CarTypeChargingLogResponse(Long id, String name) {
        this.id = id;
        this.name = name;
    }

    public static CarTypeChargingLogResponse from(CarType carType) {
        return CarTypeChargingLogResponse.builder()
                .id(carType.getId())
                .name(carType.getName())
                .build();
    }
}

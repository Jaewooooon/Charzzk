package com.ssafy.charzzk.api.service.car.response;

import com.ssafy.charzzk.domain.car.CarType;
import lombok.Builder;

public class CarTypeResponse {
    private Long id;
    private String name;
    private String image;


    @Builder
    private CarTypeResponse(Long id, String name, String image) {
        this.id = id;
        this.name = name;
        this.image = image;
    }

    public static CarTypeResponse from(CarType carType) {
        return CarTypeResponse.builder()
                .id(carType.getId())
                .name(carType.getName())
                .image(carType.getImage())
                .build();
    }
}

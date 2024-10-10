package com.ssafy.charzzk.api.controller.car.request;

import com.ssafy.charzzk.api.service.car.request.CarServiceRequest;
import jakarta.validation.constraints.NotBlank;
import jakarta.validation.constraints.NotNull;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Getter
@NoArgsConstructor
public class CarRequest {

    @NotNull(message = "차량 기종은 필수입니다.")
    private Long carTypeId;

    @NotBlank(message = "차량 번호는 필수입니다.")
    private String number;

    private String nickname;

    @Builder
    private CarRequest(Long carTypeId, String number, String nickname) {
        this.carTypeId = carTypeId;
        this.number = number;
        this.nickname = nickname;
    }

    public CarServiceRequest toServiceRequest() {
        return CarServiceRequest.builder()
                .carTypeId(carTypeId)
                .number(number)
                .nickname(nickname)
                .build();
    }



}

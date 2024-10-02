package com.ssafy.charzzk.api.controller.reservation.request;

import com.ssafy.charzzk.api.service.reservation.request.ReservationServiceRequest;
import jakarta.validation.constraints.NotNull;
import lombok.Builder;
import lombok.Getter;

@Getter
public class ReservationRequest {

    @NotNull(message = "차량 ID는 필수 값입니다.")
    private Long carId;
    @NotNull(message = "주차장 ID는 필수 값입니다.")
    private Long parkingLotId;
    private boolean fullCharge;
    private int time;

    @Builder
    private ReservationRequest(Long carId, Long parkingLotId, boolean fullCharge, int time) {
        this.carId = carId;
        this.parkingLotId = parkingLotId;
        this.fullCharge = fullCharge;
        this.time = time;
    }

    public ReservationServiceRequest toServiceRequest() {
        return ReservationServiceRequest.builder()
                .carId(carId)
                .parkingLotId(parkingLotId)
                .fullCharge(fullCharge)
                .time(time)
                .build();
    }

}

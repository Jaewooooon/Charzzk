package com.ssafy.charzzk.api.controller.reservation.request;

import com.ssafy.charzzk.api.service.reservation.request.ReservationServiceRequest;
import jakarta.validation.constraints.NotNull;
import lombok.Builder;
import lombok.Getter;

@Getter
public class ReservationRequest {

    @NotNull(message = "주차칸 ID는 필수 값입니다.")
    private Long parkingSpotId;
    @NotNull(message = "차량 ID는 필수 값입니다.")
    private Long carId;
    @NotNull(message = "주차장 ID는 필수 값입니다.")
    private Long parkingLotId;
    private boolean fullCharge;
    private int time;

    @Builder
    private ReservationRequest(Long parkingSpotId, Long carId, Long parkingLotId, boolean fullCharge, int time) {
        this.parkingSpotId = parkingSpotId;
        this.carId = carId;
        this.parkingLotId = parkingLotId;
        this.fullCharge = fullCharge;
        this.time = time;
    }

    public ReservationServiceRequest toServiceRequest() {
        return ReservationServiceRequest.builder()
                .parkingSpotId(parkingSpotId)
                .carId(carId)
                .parkingLotId(parkingLotId)
                .fullCharge(fullCharge)
                .time(time)
                .build();
    }

}

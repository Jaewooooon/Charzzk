package com.ssafy.charzzk.api.controller.parkinglot.request;

import com.ssafy.charzzk.api.service.parkinglot.request.ParkingLotListServiceRequest;
import jakarta.validation.constraints.NotNull;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Getter
@NoArgsConstructor
public class ParkingLotListRequest {

    @NotNull(message = "위도는 필수입니다.")
    private Double latitude;

    @NotNull(message = "경도는 필수입니다.")
    private Double longitude;

    @Builder
    private ParkingLotListRequest(Double latitude, Double longitude) {
        this.latitude = latitude;
        this.longitude = longitude;
    }

    public ParkingLotListServiceRequest toServiceRequest() {
        return ParkingLotListServiceRequest.builder()
            .latitude(latitude)
            .longitude(longitude)
            .build();
    }

}

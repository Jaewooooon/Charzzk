package com.ssafy.charzzk.api.service.parkinglot.request;


import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Getter
@NoArgsConstructor
public class ParkingLotListServiceRequest {

    private Double latitude;
    private Double longitude;

    @Builder
    private ParkingLotListServiceRequest(Double latitude, Double longitude) {
        this.latitude = latitude;
        this.longitude = longitude;
    }

}

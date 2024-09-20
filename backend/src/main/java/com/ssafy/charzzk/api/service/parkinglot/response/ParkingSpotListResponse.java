package com.ssafy.charzzk.api.service.parkinglot.response;

import com.ssafy.charzzk.domain.parkinglot.ParkingSpot;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

import java.util.List;

@Getter
@NoArgsConstructor
public class ParkingSpotListResponse {

    private Long id;
    private String name;

    @Builder
    private ParkingSpotListResponse(Long id, String name) {
        this.id = id;
        this.name = name;
    }

    public static ParkingSpotListResponse of(ParkingSpot parkingSpot) {
        return ParkingSpotListResponse.builder()
                .id(parkingSpot.getId())
                .name(parkingSpot.getName())
                .build();
    }
}

package com.ssafy.charzzk.api.service.parkinglot.response;

import com.ssafy.charzzk.domain.parkinglot.ParkingLot;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

import java.util.List;

@Getter
@NoArgsConstructor
public class ParkingLotResponse {

    private Long id;
    private String name;
    private String parkingMapImage;
    private List<ParkingSpotListResponse> parkingSpots;

    @Builder
    private ParkingLotResponse(Long id, String name, String parkingMapImage, List<ParkingSpotListResponse> parkingSpots) {
        this.id = id;
        this.name = name;
        this.parkingMapImage = parkingMapImage;
        this.parkingSpots = parkingSpots;
    }

    public static ParkingLotResponse of(ParkingLot parkingLot) {
        return ParkingLotResponse.builder()
            .id(parkingLot.getId())
            .name(parkingLot.getName())
            .parkingMapImage(parkingLot.getParkingMapImage())
            .parkingSpots(parkingLot.getParkingSpots().stream()
                    .map(ParkingSpotListResponse::of)
                    .toList())
            .build();
    }
}

package com.ssafy.charzzk.api.service.parkinglot.response;

import com.ssafy.charzzk.domain.parkinglot.Location;
import com.ssafy.charzzk.domain.parkinglot.ParkingLot;
import lombok.Builder;
import lombok.Getter;

@Getter
public class ParkingLotResponse {

    private Long id;
    private String name;
    private Location location;
    private String image;
    private String parkingMapImage;
    private Double distance;

    @Builder
    private ParkingLotResponse(Long id, String name, Location location, String image, String parkingMapImage, Double distance) {
        this.id = id;
        this.name = name;
        this.location = location;
        this.image = image;
        this.parkingMapImage = parkingMapImage;
        this.distance = distance;
    }

    public static ParkingLotResponse of(ParkingLot parkingLot, double distance) {
        return ParkingLotResponse.builder()
            .id(parkingLot.getId())
            .name(parkingLot.getName())
            .location(parkingLot.getLocation())
            .image(parkingLot.getImage())
            .parkingMapImage(parkingLot.getParkingMapImage())
            .distance(distance)
            .build();
    }
}

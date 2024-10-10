package com.ssafy.charzzk.api.service.parkinglot.response;

import com.ssafy.charzzk.domain.parkinglot.Location;
import com.ssafy.charzzk.domain.parkinglot.ParkingLot;
import lombok.Builder;
import lombok.Getter;

@Getter
public class ParkingLotListResponse {

    private Long id;
    private String name;
    private Location location;
    private String image;
    private Double distance;
    private int waitingTime;

    @Builder
    private ParkingLotListResponse(Long id, String name, Location location, String image, Double distance, int waitingTime) {
        this.id = id;
        this.name = name;
        this.location = location;
        this.image = image;
        this.distance = distance;
        this.waitingTime = waitingTime;
    }

    public static ParkingLotListResponse of(ParkingLot parkingLot, double distance, int waitingTime) {
        return ParkingLotListResponse.builder()
            .id(parkingLot.getId())
            .name(parkingLot.getName())
            .location(parkingLot.getLocation())
            .image(parkingLot.getImage())
            .distance(distance)
            .waitingTime(waitingTime)
            .build();
    }
}

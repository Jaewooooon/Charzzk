package com.ssafy.charzzk.api.service.parkinglot.response;

import com.ssafy.charzzk.domain.parkinglot.ParkingLot;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Getter
@NoArgsConstructor
public class ParkingLotReportResponse {

    private Long id;
    private String name;

    @Builder
    private ParkingLotReportResponse(Long id, String name) {
        this.id = id;
        this.name = name;
    }

    public static ParkingLotReportResponse of(ParkingLot parkingLot) {
        return ParkingLotReportResponse.builder()
                .id(parkingLot.getId())
                .name(parkingLot.getName())
                .build();
    }

}

package com.ssafy.charzzk.api.controller.parkinglot;


import com.ssafy.charzzk.api.ApiResponse;
import com.ssafy.charzzk.api.controller.parkinglot.request.ParkingLotListRequest;
import com.ssafy.charzzk.api.service.parkinglot.ParkingLotService;
import com.ssafy.charzzk.api.service.parkinglot.response.ParkingLotListResponse;
import com.ssafy.charzzk.api.service.parkinglot.response.ParkingLotResponse;
import jakarta.validation.Valid;
import lombok.RequiredArgsConstructor;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PathVariable;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RestController;

import java.util.List;

@RequiredArgsConstructor
@RestController
public class ParkingLotController {

    private final ParkingLotService parkingLotService;

    @GetMapping("/api/v1/parking-lot")
    public ApiResponse<List<ParkingLotListResponse>> getParkingLotList(
            @Valid @RequestBody ParkingLotListRequest request
    ) {
        return ApiResponse.ok(parkingLotService.getParkingLotList(request));
    }

    @GetMapping("/api/v1/parking-lot/{parkingLotId}")
    public ApiResponse<ParkingLotResponse> getParkingList(
            @PathVariable Long parkingLotId
    ) {
        return ApiResponse.ok(parkingLotService.getParkingLot(parkingLotId));
    }
}

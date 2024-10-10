package com.ssafy.charzzk.api.controller.parkinglot;


import com.ssafy.charzzk.api.ApiResponse;
import com.ssafy.charzzk.api.controller.parkinglot.request.ParkingLotListRequest;
import com.ssafy.charzzk.api.service.charger.response.ChargerResponse;
import com.ssafy.charzzk.api.service.parkinglot.ParkingLotService;
import com.ssafy.charzzk.api.service.parkinglot.response.ParkingLotListResponse;
import com.ssafy.charzzk.api.service.parkinglot.response.ParkingLotResponse;
import jakarta.validation.Valid;
import lombok.RequiredArgsConstructor;
import org.springframework.web.bind.annotation.*;

import java.time.LocalDateTime;
import java.util.List;

@RequiredArgsConstructor
@RestController
public class ParkingLotController {

    private final ParkingLotService parkingLotService;

    @GetMapping("/api/v1/parking-lot")
    public ApiResponse<List<ParkingLotListResponse>> getParkingLotList(
            @RequestParam(value = "latitude", defaultValue = "0.0") Double latitude,
            @RequestParam(value = "longitude", defaultValue = "0.0") Double longitude,
            @RequestParam(value = "q", defaultValue = "", required = false) String keyword,
            @RequestParam(value = "sort", defaultValue = "distance") String sort
    ) {
        LocalDateTime now = LocalDateTime.now();
        return ApiResponse.ok(parkingLotService.getParkingLotList(latitude, longitude, keyword, sort, now));
    }

    @GetMapping("/api/v1/parking-lot/{parkingLotId}")
    public ApiResponse<ParkingLotResponse> getParkingList(
            @PathVariable Long parkingLotId
    ) {
        return ApiResponse.ok(parkingLotService.getParkingLot(parkingLotId));
    }

    @GetMapping("/api/v1/parking-lot/{parkingLotId}/chargers")
    public ApiResponse<List<ChargerResponse>> getChargerList(
            @PathVariable Long parkingLotId
    ) {
        return ApiResponse.ok(parkingLotService.getChargerList(parkingLotId));
    }

}

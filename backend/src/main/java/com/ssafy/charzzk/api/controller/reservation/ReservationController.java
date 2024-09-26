package com.ssafy.charzzk.api.controller.reservation;

import com.ssafy.charzzk.api.ApiResponse;
import com.ssafy.charzzk.api.service.reservation.ReservationService;
import com.ssafy.charzzk.api.service.response.ReservationCheckTimeResponse;
import lombok.RequiredArgsConstructor;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.RequestParam;
import org.springframework.web.bind.annotation.RestController;

import java.time.LocalDateTime;

@RequiredArgsConstructor
@RestController
public class ReservationController {

    private final ReservationService reservationService;

    @GetMapping("/api/v1/reservations/check-time")
    public ApiResponse<ReservationCheckTimeResponse> checkTime(
            @RequestParam(value = "parkingLotId") Long parkingLotId,
            @RequestParam(value = "carId") Long carId,
            @RequestParam(value = "fullCharge") boolean fullCharge,
            @RequestParam(value = "time") int time
    ) {
        LocalDateTime now = LocalDateTime.now();
        return ApiResponse.ok(reservationService.checkTime(parkingLotId, carId, fullCharge, time, now));
    }
}

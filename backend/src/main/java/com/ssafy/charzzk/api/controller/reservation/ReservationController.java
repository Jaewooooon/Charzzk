package com.ssafy.charzzk.api.controller.reservation;

import com.ssafy.charzzk.api.ApiResponse;
import com.ssafy.charzzk.api.controller.reservation.request.ReservationConfirmRequest;
import com.ssafy.charzzk.api.controller.reservation.request.ReservationRequest;
import com.ssafy.charzzk.api.service.reservation.ReservationService;
import com.ssafy.charzzk.api.service.reservation.response.ReservationResponse;
import com.ssafy.charzzk.core.annotation.CurrentUser;
import com.ssafy.charzzk.domain.reservation.Reservation;
import com.ssafy.charzzk.domain.user.User;
import jakarta.validation.Valid;
import lombok.RequiredArgsConstructor;
import org.springframework.web.bind.annotation.*;

import java.time.LocalDateTime;

@RequiredArgsConstructor
@RestController
public class ReservationController {

    private final ReservationService reservationService;

    @PostMapping("/api/v1/reservations")
    public ApiResponse<ReservationResponse> createReservation(
            @CurrentUser User user,
            @Valid @RequestBody ReservationRequest request
    ) {
        LocalDateTime now = LocalDateTime.now();
        Reservation reservation = reservationService.create(user, request.toServiceRequest(), now);

        return ApiResponse.ok(reservationService.getReservation(reservation.getId()));
    }

    @PatchMapping("/api/v1/reservations/{reservationId}")
    public ApiResponse<ReservationResponse> confirmReservation(
            @CurrentUser User user,
            @Valid @RequestBody ReservationConfirmRequest request
    ) {
        Reservation reservation = reservationService.confirm(user, request.toServiceRequest());

        return ApiResponse.ok(reservationService.getReservation(reservation.getId()));
    }

}

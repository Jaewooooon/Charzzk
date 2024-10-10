package com.ssafy.charzzk.api.controller.reservation;

import com.ssafy.charzzk.api.ApiResponse;
import com.ssafy.charzzk.api.controller.reservation.request.ReservationRequest;
import com.ssafy.charzzk.api.service.reservation.ReservationService;
import com.ssafy.charzzk.api.service.reservation.response.ReservationQueueResponse;
import com.ssafy.charzzk.api.service.reservation.response.ReservationResponse;
import com.ssafy.charzzk.core.annotation.CurrentUser;
import com.ssafy.charzzk.domain.reservation.Reservation;
import com.ssafy.charzzk.domain.user.User;
import jakarta.validation.Valid;
import lombok.RequiredArgsConstructor;
import org.springframework.web.bind.annotation.*;

import java.time.LocalDateTime;
import java.util.List;

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
            @PathVariable Long reservationId
    ) {
        Reservation reservation = reservationService.confirm(user, reservationId);

        return ApiResponse.ok(reservationService.getReservation(reservation.getId()));
    }

    @DeleteMapping("/api/v1/reservations/{reservationId}")
    public ApiResponse<ReservationResponse> cancelReservation(
            @CurrentUser User user,
            @PathVariable Long reservationId
    ) {
        Reservation reservation = reservationService.cancel(user, reservationId);

        return ApiResponse.ok(reservationService.getReservation(reservation.getId()));
    }


    /**
     * 큐에 들어있는 예약 목록 반환
     * @return
     */
    @GetMapping("/api/v1/reservations")
    public ApiResponse<List<ReservationQueueResponse>> getReservation() {
        return ApiResponse.ok(reservationService.getReservations());
    }

    /**
     * 큐에 초기화
     * @return
     */
    @DeleteMapping("/api/v1/reservations")
    public ApiResponse<List<ReservationQueueResponse>> deleteReservation() {
        reservationService.deleteReservations();
        return ApiResponse.ok(null);
    }

}


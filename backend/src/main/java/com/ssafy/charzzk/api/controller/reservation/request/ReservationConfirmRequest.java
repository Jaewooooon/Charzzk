package com.ssafy.charzzk.api.controller.reservation.request;

import com.ssafy.charzzk.api.service.reservation.request.ReservationConfirmServiceRequest;
import jakarta.validation.constraints.NotNull;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Getter
@NoArgsConstructor
public class ReservationConfirmRequest {

    @NotNull(message = "예약 ID는 필수 입력 값입니다.")
    private Long reservationId;

    @Builder
    private ReservationConfirmRequest(Long reservationId) {
        this.reservationId = reservationId;
    }

    public ReservationConfirmServiceRequest toServiceRequest() {
        return ReservationConfirmServiceRequest.builder()
                .reservationId(reservationId)
                .build();
    }
}

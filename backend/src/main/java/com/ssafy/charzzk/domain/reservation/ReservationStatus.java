package com.ssafy.charzzk.domain.reservation;

public enum ReservationStatus {
    PENDING,            // 예약 확정 전
    WAITING,    // 충전 전, 확정 후
    CHARGING,           // 충전 중
    DONE,            // 충전 완료
    CANCELED;      // 예약 취소
}

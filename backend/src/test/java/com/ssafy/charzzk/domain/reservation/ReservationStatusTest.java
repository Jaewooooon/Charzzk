package com.ssafy.charzzk.domain.reservation;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class ReservationStatusTest {

    @DisplayName("예약이 대기중이면 True를 반환하고 아니면 False를 반환한다")
    @Test
    void isWaiting() {
        assertTrue(ReservationStatus.WAITING.isWaiting());
        assertTrue(ReservationStatus.CHARGING.isWaiting());
        assertFalse(ReservationStatus.PENDING.isWaiting());
        assertFalse(ReservationStatus.DONE.isWaiting());
        assertFalse(ReservationStatus.CANCELED.isWaiting());
    }

}

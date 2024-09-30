package com.ssafy.charzzk.domain.charger;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

import static org.assertj.core.api.Assertions.assertThat;


class ChargerStatusTest {

    @DisplayName("이용 가능 상태는 다음과 같다.")
    @Test
    public void isAvailable() {
        assertThat(ChargerStatus.WAITING.isAvailable()).isTrue();
        assertThat(ChargerStatus.MOVING_TO_CAR.isAvailable()).isTrue();
        assertThat(ChargerStatus.RETURNING_TO_CAR.isAvailable()).isTrue();
        assertThat(ChargerStatus.CAR_CHARGING.isAvailable()).isTrue();
        assertThat(ChargerStatus.CHARGER_CHARGING.isAvailable()).isTrue();
    }

    @DisplayName("이용 불가능 상태는 다음과 같다.")
    @Test
    public void isNotAvailable() {
        assertThat(ChargerStatus.ERROR.isAvailable()).isFalse();
        assertThat(ChargerStatus.MAINTENANCE.isAvailable()).isFalse();
    }

}

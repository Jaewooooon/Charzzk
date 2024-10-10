package com.ssafy.charzzk.domain.charger;

import com.ssafy.charzzk.domain.parkinglot.ParkingLot;
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

    @DisplayName("대기 상태인지 확인한다.")
    @Test
    public void isWaiting() {
        assertThat(ChargerStatus.WAITING.isWaiting()).isTrue();
    }
    
    @DisplayName("충전을 멈추면 대기 상태가 된다.")
    @Test
    public void stopCharge() {
        Charger charger = Charger.builder()
                .status(ChargerStatus.CAR_CHARGING)
                .build();

        charger.stopCharge();

        assertThat(charger.getStatus()).isEqualTo(ChargerStatus.WAITING);
    }

    @DisplayName("충전이 끝나면 대기 상태가 된다.")
    @Test
    public void chargeComplete() {
        Charger charger = Charger.builder()
                .status(ChargerStatus.CAR_CHARGING)
                .build();

        charger.chargeComplete();

        assertThat(charger.getStatus()).isEqualTo(ChargerStatus.WAITING);
    }

    @DisplayName("충전기에 주차장을 등록한다.")
    @Test
    public void setParkingLot() {
        Charger charger = Charger.builder()
                .build();

        ParkingLot parkingLot = ParkingLot.builder()
                .name("첨단 주차장")
                .build();

        charger.setParkingLot(parkingLot);

        assertThat(charger.getParkingLot()).isEqualTo(parkingLot);
        assertThat(parkingLot.getChargers()).contains(charger);
    }

}

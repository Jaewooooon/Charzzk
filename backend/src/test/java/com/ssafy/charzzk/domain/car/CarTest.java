package com.ssafy.charzzk.domain.car;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

import static org.assertj.core.api.Assertions.assertThat;

class CarTest {

    @DisplayName("차량정보 업데이트")
    @Test
    void updateCar() {
        // given
        Car car = Car.builder()
                .carType(CarType.builder().name("차종1").build())
                .number("12다1234")
                .nickname("차량1")
                .build();
        CarType newCarType = CarType.builder().name("차종2").build();

        // when
        car.updateCar(newCarType, "12다1235", "차량2");

        // then
        assertThat(car)
                .extracting("carType", "number", "nickname")
                .containsExactly(newCarType, "12다1235", "차량2");
    }

    @DisplayName("차량 배터리 충전")
    @Test
    void chargeBattery() {
        // given
        Car car = Car.builder()
                .carType(CarType.builder().name("차종1").build())
                .number("12다1234")
                .nickname("차량1")
                .build();

        // when
        car.chargeBattery(50);

        // then
        assertThat(car.getBattery()).isEqualTo(50);
    }

    @DisplayName("차량 배터리 충전 완료")
    @Test
    void chargeComplete() {
        // given
        Car car = Car.builder()
                .carType(CarType.builder().name("차종1").build())
                .number("12다1234")
                .nickname("차량1")
                .isCharging(true)
                .build();

        // when
        car.chargeComplete();

        // then
        assertThat(car.isCharging()).isFalse();
    }
}
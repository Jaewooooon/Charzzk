package com.ssafy.charzzk.api.service.reservation;

import com.ssafy.charzzk.domain.car.Car;
import com.ssafy.charzzk.domain.car.CarType;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.api.Assertions.*;

class ChargeTimeCalculatorTest {

    @DisplayName("완충인 경우 충전시간을 계산한다.")
    @Test
    void calculateFullChargeTime() {
        // given
        CarType carType = CarType.builder()
                .id(1L)
                .name("아이오닉5")
                .build();

        Car car = Car.builder()
                .carType(carType)
                .number("12다3445")
                .nickname("붕붕이")
                .battery(30)
                .build();

        int time = 0;
        boolean fullCharge = true;

        // when
        int duration = ChargeTimeCalculator.calculate(car, fullCharge, time);

        // then
        assertThat(duration).isEqualTo(70);
    }

    @DisplayName("충전시간을 설정하고 충전시간동안 완충되지 않는경우 충전시간을 계산한다.")
    @Test
    public void calculateChargeTimeWithTimeSet() {
        CarType carType = CarType.builder()
                .id(1L)
                .name("아이오닉5")
                .build();

        Car car = Car.builder()
                .carType(carType)
                .number("12다3445")
                .nickname("붕붕이")
                .battery(30)
                .build();

        int time = 30;
        boolean fullCharge = false;

        // when
        int duration = ChargeTimeCalculator.calculate(car, fullCharge, time);

        // then
        assertThat(duration).isEqualTo(30);
    }


    @DisplayName("충전시간을 설정하고 충전시간동안 완충되는 경우 충전시간을 계산한다.")
    @Test
    public void calculateFullChargeTimeWithTimeSet() {
        CarType carType = CarType.builder()
                .id(1L)
                .name("아이오닉5")
                .build();

        Car car = Car.builder()
                .carType(carType)
                .number("12다3445")
                .nickname("붕붕이")
                .battery(90)
                .build();

        int time = 30;
        boolean fullCharge = false;

        // when
        int duration = ChargeTimeCalculator.calculate(car, fullCharge, time);

        // then
        assertThat(duration).isEqualTo(10);
    }

}

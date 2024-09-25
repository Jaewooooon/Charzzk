package com.ssafy.charzzk.api.service.reservation;

import com.ssafy.charzzk.domain.car.Car;

public class ChargeTimeCalculator {

    private static final int CHARGE_SPEED_PER_MINUTE = 1;

    public static int calculate(Car car, boolean fullCharge, int time) {
        int currentBattery = car.getBattery();
        int duration = (100 - currentBattery) / CHARGE_SPEED_PER_MINUTE;

        if (fullCharge) {
            return duration;
        }

        return Math.min(duration, time);
    }
}

package com.ssafy.charzzk.api.service.reservation;

import com.ssafy.charzzk.core.exception.BaseException;
import com.ssafy.charzzk.core.exception.ErrorCode;
import com.ssafy.charzzk.core.util.ChargeTimeCalculator;
import com.ssafy.charzzk.domain.car.Car;
import com.ssafy.charzzk.domain.charger.Charger;
import com.ssafy.charzzk.domain.charger.ChargerRepository;
import com.ssafy.charzzk.domain.parkinglot.ParkingLot;
import com.ssafy.charzzk.domain.reservation.Reservation;
import com.ssafy.charzzk.domain.reservation.ReservationCacheRepository;
import com.ssafy.charzzk.domain.reservation.ReservationConst;
import com.ssafy.charzzk.domain.reservation.ReservationRepository;
import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Component;
import org.springframework.transaction.annotation.Transactional;

import java.time.LocalDateTime;
import java.util.List;

@Transactional(readOnly = true)
@RequiredArgsConstructor
@Component
public class ReservationManager {

    private final ReservationRepository reservationRepository;
    private final ReservationCacheRepository reservationCacheRepository;

    @Transactional
    public Reservation createReservation(ParkingLot parkingLot, Car car, int time, boolean fullCharge, LocalDateTime now) {

        List<Charger> chargers = parkingLot.getChargers();

        Charger charger = chargers.stream()
                .filter(c -> c.getStatus().isAvailable())
                .min((c1, c2) -> c1.getLastReservedTime().isBefore(c2.getLastReservedTime()) ? -1 : 1)
                .orElseThrow(() -> new BaseException(ErrorCode.NO_AVAILABLE_CHARGER));

        LocalDateTime startTime = charger.getLastReservedTime().isAfter(now) ? charger.getLastReservedTime() : now;
        LocalDateTime endTime = startTime.plusMinutes(ChargeTimeCalculator.calculate(car, fullCharge, time));
        charger.updateLastReservedTime(startTime);

        Reservation reservation = Reservation.create(car, charger, startTime, endTime);

        reservationRepository.save(reservation);
        reservationCacheRepository.createReservationGracePeriod(reservation.getId(), charger.getId(), ReservationConst.gracePeriod);

        return reservation;
    }

}

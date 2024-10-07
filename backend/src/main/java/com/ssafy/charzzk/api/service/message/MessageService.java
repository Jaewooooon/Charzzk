package com.ssafy.charzzk.api.service.message;


import com.ssafy.charzzk.api.service.reservation.ReservationManager;
import com.ssafy.charzzk.core.exception.BaseException;
import com.ssafy.charzzk.core.exception.ErrorCode;
import com.ssafy.charzzk.domain.car.Car;
import com.ssafy.charzzk.domain.car.CarRepository;
import com.ssafy.charzzk.domain.charger.Charger;
import com.ssafy.charzzk.domain.charginglog.ChargingLog;
import com.ssafy.charzzk.domain.charginglog.ChargingLogRepository;
import com.ssafy.charzzk.domain.reservation.Reservation;
import com.ssafy.charzzk.domain.reservation.ReservationRepository;
import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

@Transactional(readOnly = true)
@RequiredArgsConstructor
@Service
public class MessageService {

    private final ReservationManager reservationManager;

    private final ReservationRepository reservationRepository;
    private final ChargingLogRepository chargingLogRepository;

    @Transactional
    public void updateBatteryStatus(Long reservationId, Integer batteryLevel) {
        Reservation reservation = reservationRepository.findByIdWithCarAndCharger(reservationId)
                .orElseThrow(() -> new BaseException(ErrorCode.RESERVATION_NOT_FOUND));

        Car car = reservation.getCar();

        car.chargeBattery(batteryLevel);
    }

    @Transactional
    public ChargingLog chargeComplete(Long reservationId) {
        Reservation reservation = reservationRepository.findByIdWithCarAndCharger(reservationId)
                .orElseThrow(() -> new BaseException(ErrorCode.RESERVATION_NOT_FOUND));

        reservation.chargeComplete();

        // reservationManager에서 충전기의 다음 예약 진행하기
        reservationManager.executeNextReservation(reservation.getCharger());

        ChargingLog chargingLog = ChargingLog.of(reservation);
        return chargingLogRepository.save(chargingLog);
        // TODO 알림
    }
}

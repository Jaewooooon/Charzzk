package com.ssafy.charzzk.api.service.reservation;

import com.ssafy.charzzk.api.service.response.ReservationCheckTimeResponse;
import com.ssafy.charzzk.core.exception.BaseException;
import com.ssafy.charzzk.core.exception.ErrorCode;
import com.ssafy.charzzk.domain.car.Car;
import com.ssafy.charzzk.domain.car.CarRepository;
import com.ssafy.charzzk.domain.charger.Charger;
import com.ssafy.charzzk.domain.charger.ChargerStatus;
import com.ssafy.charzzk.domain.parkinglot.ParkingLot;
import com.ssafy.charzzk.domain.parkinglot.ParkingLotRepository;
import com.ssafy.charzzk.domain.reservation.Reservation;
import com.ssafy.charzzk.domain.reservation.ReservationRepository;
import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

import java.time.LocalDateTime;
import java.util.List;
import java.util.Map;
import java.util.Optional;
@Transactional(readOnly = true)
@RequiredArgsConstructor
@Service
public class ReservationService {

    private final ParkingLotRepository parkingLotRepository;
    private final CarRepository carRepository;
    private final ReservationRepository reservationRepository;

    public ReservationCheckTimeResponse checkTime(Long parkingLotId, Long carId, boolean fullCharge, int time, LocalDateTime now) {
        ParkingLot parkingLot = parkingLotRepository.findByIdWithChargers(parkingLotId)
                .orElseThrow(() -> new BaseException(ErrorCode.PARKING_LOT_NOT_FOUND));

        List<Charger> chargers = parkingLot.getChargers();
        if (chargers.isEmpty()) {
            throw new BaseException(ErrorCode.CHARGER_NOT_EXIST_IN_PARKING_LOT);
        }

        Car car = carRepository.findById(carId)
                .orElseThrow(() -> new BaseException(ErrorCode.CAR_NOT_FOUND));

        int duration = ChargeTimeCalculator.calculate(car, fullCharge, time);

        // 충전기 중 기다리는 상태의 충전기 확인
        Optional<Charger> availableCharger = chargers.stream()
                .filter(charger -> charger.getStatus().equals(ChargerStatus.WAITING))
                .findFirst();

        // 당장 이용 가능한 충전기가 있으면
        if (availableCharger.isPresent()) {
            return ReservationCheckTimeResponse.builder()
                    .chargerId(availableCharger.get().getId())
                    .startTime(now)
                    .endTime(now.plusMinutes(duration))
                    .isPossibleNow(true)
                    .build();
        }

        // 이용 가능한 충전기가 없으면 가장 빨리 끝나는 예약을 찾는다.
        Map.Entry<Long, LocalDateTime> fastestCharger = chargers.stream()
                .map(charger -> {
                    Optional<Reservation> reservation = reservationRepository.findFirstByChargerIdOrderByEndTimeDesc(charger.getId());
                    return reservation.map(res -> Map.entry(charger.getId(), res.getEndTime()));
                })
                .filter(Optional::isPresent)
                .map(Optional::get)
                .min(Map.Entry.comparingByValue())
                .orElse(Map.entry(-1L, LocalDateTime.MAX));

        return ReservationCheckTimeResponse.builder()
                .chargerId(fastestCharger.getKey())
                .startTime(fastestCharger.getValue())
                .endTime(fastestCharger.getValue().plusMinutes(duration))
                .isPossibleNow(false)
                .build();
    }
}
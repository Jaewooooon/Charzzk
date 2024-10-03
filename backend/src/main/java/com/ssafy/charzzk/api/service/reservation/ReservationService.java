package com.ssafy.charzzk.api.service.reservation;

import com.ssafy.charzzk.api.service.reservation.request.ReservationConfirmServiceRequest;
import com.ssafy.charzzk.api.service.reservation.request.ReservationServiceRequest;
import com.ssafy.charzzk.api.service.reservation.response.ReservationResponse;
import com.ssafy.charzzk.core.exception.BaseException;
import com.ssafy.charzzk.core.exception.ErrorCode;
import com.ssafy.charzzk.core.util.ChargeTimeCalculator;
import com.ssafy.charzzk.domain.car.Car;
import com.ssafy.charzzk.domain.car.CarRepository;
import com.ssafy.charzzk.domain.charger.Charger;
import com.ssafy.charzzk.domain.charger.ChargerRepository;
import com.ssafy.charzzk.domain.parkinglot.ParkingLot;
import com.ssafy.charzzk.domain.parkinglot.ParkingLotRepository;
import com.ssafy.charzzk.domain.parkinglot.ParkingSpot;
import com.ssafy.charzzk.domain.parkinglot.ParkingSpotRepository;
import com.ssafy.charzzk.domain.reservation.Reservation;
import com.ssafy.charzzk.domain.reservation.ReservationCacheRepository;
import com.ssafy.charzzk.domain.reservation.ReservationConst;
import com.ssafy.charzzk.domain.reservation.ReservationRepository;
import com.ssafy.charzzk.domain.user.User;
import jakarta.annotation.PostConstruct;
import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

import java.time.LocalDateTime;
import java.util.List;

@Transactional(readOnly = true)
@RequiredArgsConstructor
@Service
public class ReservationService {

    private final ReservationManager reservationManager;

    private final ReservationRepository reservationRepository;
    private final ReservationCacheRepository reservationCacheRepository;
    private final ParkingLotRepository parkingLotRepository;
    private final ParkingSpotRepository parkingSpotRepository;
    private final CarRepository carRepository;
    private final ChargerRepository chargerRepository;

    @PostConstruct
    public void init() {
        List<Charger> chargers = chargerRepository.findAll();
        reservationManager.init(chargers);
    }

    public ReservationResponse getReservation(Long reservationId) {
        Reservation findReservation = reservationRepository.findByIdWithCar(reservationId)
                .orElseThrow(() -> new BaseException(ErrorCode.RESERVATION_NOT_FOUND));

        return ReservationResponse.of(findReservation);
    }

    @Transactional
    public Reservation create(User user, ReservationServiceRequest request, LocalDateTime now) {

        Car car = carRepository.findById(request.getCarId())
                .orElseThrow(() -> new BaseException(ErrorCode.CAR_NOT_FOUND));
        if (!car.getUser().equals(user)) {
            throw new BaseException(ErrorCode.FORBIDDEN);
        }

        ParkingLot parkingLot = parkingLotRepository.findByIdWithChargers(request.getParkingLotId())
                .orElseThrow(() -> new BaseException(ErrorCode.PARKING_LOT_NOT_FOUND));

        ParkingSpot parkingSpot = parkingSpotRepository.findById(request.getParkingSpotId())
                .orElseThrow(() -> new BaseException(ErrorCode.PARKING_SPOT_NOT_FOUND));

        // 옮기기
        List<Charger> chargers = parkingLot.getChargers();

        Charger charger = chargers.stream()
                .filter(c -> c.getStatus().isAvailable())
                .min((c1, c2) -> c1.getLastReservedTime().isBefore(c2.getLastReservedTime()) ? -1 : 1)
                .orElseThrow(() -> new BaseException(ErrorCode.NO_AVAILABLE_CHARGER));

        LocalDateTime startTime = charger.getLastReservedTime().isAfter(now) ? charger.getLastReservedTime() : now;
        LocalDateTime endTime = startTime.plusMinutes(ChargeTimeCalculator.calculate(car, request.isFullCharge(), request.getTime()));
        charger.updateLastReservedTime(startTime);

        Reservation reservation = Reservation.create(car, parkingSpot, charger, startTime, endTime);
        reservationRepository.save(reservation);
        reservationCacheRepository.createReservationGracePeriod(reservation.getId(), charger.getId(), ReservationConst.gracePeriod);

        reservationManager.createReservation(reservation);

        return reservation;
    }

    @Transactional
    public void timeout(Long reservationId) {
        reservationRepository.deleteById(reservationId);

        // TODO : 예약시간 스케줄링
//        reservationManager.deleteReservation(reservationId);
    }

    @Transactional
    public Reservation confirm(User user, ReservationConfirmServiceRequest request) {

        Reservation reservation = reservationRepository.findByIdWithCarAndCharger(request.getReservationId())
                .orElseThrow(() -> new BaseException(ErrorCode.RESERVATION_NOT_FOUND));

        if (!reservation.getCar().getUser().equals(user)) {
            throw new BaseException(ErrorCode.FORBIDDEN);
        }

        if (!reservationCacheRepository.existsReservationGracePeriod(reservation.getId(), reservation.getCharger().getId())) {
            throw new BaseException(ErrorCode.RESERVATION_CONFIRM_TIMEOUT);
        }

        reservation.confirm();

        reservationManager.confirmReservation(reservation);

        return reservation;
    }

}

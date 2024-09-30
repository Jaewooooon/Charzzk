package com.ssafy.charzzk.api.service.reservation;

import com.ssafy.charzzk.api.service.reservation.request.ReservationServiceRequest;
import com.ssafy.charzzk.api.service.reservation.response.ReservationResponse;
import com.ssafy.charzzk.core.exception.BaseException;
import com.ssafy.charzzk.core.exception.ErrorCode;
import com.ssafy.charzzk.core.util.ChargeTimeCalculator;
import com.ssafy.charzzk.domain.car.Car;
import com.ssafy.charzzk.domain.car.CarRepository;
import com.ssafy.charzzk.domain.charger.Charger;
import com.ssafy.charzzk.domain.parkinglot.ParkingLot;
import com.ssafy.charzzk.domain.parkinglot.ParkingLotRepository;
import com.ssafy.charzzk.domain.reservation.Reservation;
import com.ssafy.charzzk.domain.reservation.ReservationCacheRepository;
import com.ssafy.charzzk.domain.reservation.ReservationRepository;
import com.ssafy.charzzk.domain.user.User;
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
    private final ParkingLotRepository parkingLotRepository;
    private final CarRepository carRepository;

    public ReservationResponse getReservation(Long reservationId) {
        Reservation findReservation = reservationRepository.findByIdWithCar(reservationId)
                .orElseThrow(() -> new BaseException(ErrorCode.RESERVATION_NOT_FOUND));

        return ReservationResponse.of(findReservation);
    }

    @Transactional
    public Long create(User user, ReservationServiceRequest request, LocalDateTime now) {

        Car car = carRepository.findById(request.getCarId())
                .orElseThrow(() -> new BaseException(ErrorCode.CAR_NOT_FOUND));
        if (!car.getUser().equals(user)) {
            throw new BaseException(ErrorCode.FORBIDDEN);
        }

        ParkingLot parkingLot = parkingLotRepository.findByIdWithChargers(request.getParkingLotId())
                .orElseThrow(() -> new BaseException(ErrorCode.PARKING_LOT_NOT_FOUND));

        Reservation reservation = reservationManager.createReservation(parkingLot, car, request.getTime(), request.isFullCharge(), now);

        return reservation.getId();
    }

    @Transactional
    public void timeout(Long reservationId) {
        reservationRepository.deleteById(reservationId);

        // TODO : 예약시간 스케줄링
//        reservationManager.deleteReservation(reservationId);
    }


}

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
import com.ssafy.charzzk.domain.user.User;
import jakarta.annotation.PostConstruct;
import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Component;
import org.springframework.transaction.annotation.Transactional;

import java.time.LocalDateTime;
import java.util.ArrayDeque;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.concurrent.ConcurrentHashMap;

@Transactional(readOnly = true)
@RequiredArgsConstructor
@Component
public class ReservationManager {

    Map<Long, Queue<Reservation>> reservationQueueMap = new ConcurrentHashMap<>();

    private final ReservationRepository reservationRepository;
    private final ReservationCacheRepository reservationCacheRepository;
    private final ChargerRepository chargerRepository;

    @PostConstruct
    public void init() {
        List<Charger> chargers = chargerRepository.findAll();

        for (Charger charger : chargers) {
            reservationQueueMap.put(charger.getId(), new LinkedList<>());
        }
    }

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

    @Transactional
    public Reservation confirmReservation(User user, Long reservationId) {

        Reservation reservation = reservationRepository.findByIdWithCarAndCharger(reservationId)
                .orElseThrow(() -> new BaseException(ErrorCode.RESERVATION_NOT_FOUND));

        if (!reservation.getCar().getUser().equals(user)) {
            throw new BaseException(ErrorCode.FORBIDDEN);
        }

        if (!reservationCacheRepository.existsReservationGracePeriod(reservation.getId(), reservation.getCharger().getId())) {
            throw new BaseException(ErrorCode.RESERVATION_CONFIRM_TIMEOUT);
        }

        reservation.confirm();
        Queue<Reservation> reservations = reservationQueueMap.get(reservation.getCharger().getId());
        reservations.add(reservation);

        // TODO : 임베디드서버 api요청 보내서 로봇이 이용 가능한지 확인
        // 이용 가능하면 큐에서 꺼내서 예약정보 보내기
        // 이용 불가능하면 스킵

        return reservation;
    }

}

package com.ssafy.charzzk.api.service.reservation;

import com.ssafy.charzzk.core.apiclient.ChargerClient;
import com.ssafy.charzzk.core.apiclient.request.ChargerCancelRequest;
import com.ssafy.charzzk.core.apiclient.request.ChargerCommandRequest;
import com.ssafy.charzzk.core.exception.BaseException;
import com.ssafy.charzzk.core.exception.ErrorCode;
import com.ssafy.charzzk.core.util.ChargeTimeCalculator;
import com.ssafy.charzzk.domain.charger.Charger;
import com.ssafy.charzzk.domain.parkinglot.ParkingLot;
import com.ssafy.charzzk.domain.reservation.Reservation;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.stereotype.Component;

import java.time.Duration;
import java.time.LocalDateTime;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentLinkedDeque;
import java.util.concurrent.ConcurrentLinkedQueue;

@RequiredArgsConstructor
@Component
@Getter
@Slf4j
public class ReservationManager {

    private Map<Long, Deque<Reservation>> reservationQueueMap = new HashMap<>();
    private Queue<Reservation> sequenceQueue = new LinkedList<>();

    private final Object lock = new Object();

    private final ChargerClient chargerClient;

    public void init(List<Charger> chargers) {
        for (Charger charger : chargers) {
            reservationQueueMap.put(charger.getId(), new ArrayDeque<>());
        }
    }

    public void createReservation(Reservation reservation) {
        synchronized (lock) {
            reservationQueueMap.get(reservation.getCharger().getId()).add(reservation);
            sequenceQueue.add(reservation);
        }
    }

    public void confirmReservation(Reservation reservation) {

        synchronized (lock) {
            Queue<Reservation> reservations = reservationQueueMap.get(reservation.getCharger().getId());

            reservations.stream().filter(r -> r.equals(reservation))
                    .findFirst()
                    .orElseThrow(() -> new BaseException(ErrorCode.RESERVATION_NOT_FOUND))
                    .confirm();

            if (reservations.peek().equals(reservation) && reservation.getCharger().getStatus().isWaiting()) {
                chargerClient.command(ChargerCommandRequest.of(reservation));
                reservations.poll();

                sequenceQueue.remove(reservation);

                reservation.start();
            }
        }
    }

    public Reservation cancelReservation(Reservation reservation) {

        synchronized (lock) {
            Queue<Reservation> reservations = reservationQueueMap.get(reservation.getCharger().getId());

            if (reservation.isWaiting()) { // 대기중이면 취소
                sequenceQueue.remove(reservation);
            } else if (reservation.isCharging()) { // 충전중이면 중지 명령 내리기
                chargerClient.cancel(ChargerCancelRequest.of(reservation));
                reservation.getCharger().stopCharge();
            } else {
                throw new BaseException(ErrorCode.CANNOT_CANCEL_RESERVATION);
            }

            reservations.remove(reservation);
            reservation.cancel();

            // 예약 재분배
            relocate(reservation, LocalDateTime.now());

            executeNextReservation(reservation.getCharger());

            return reservation;
        }
    }

    public void timeout(Reservation reservation) {
        synchronized (lock) {
            Queue<Reservation> reservations = reservationQueueMap.get(reservation.getCharger().getId());
            reservations.remove(reservation);
            sequenceQueue.remove(reservation);

            relocate(reservation, LocalDateTime.now());
        }
    }


    public void executeNextReservation(Charger charger) {
        synchronized (lock) {
            Queue<Reservation> reservations = reservationQueueMap.get(charger.getId());

            Reservation reservation = reservations.peek();

            if (reservation != null && reservation.isWaiting()) {
                chargerClient.command(ChargerCommandRequest.of(reservation));

                reservations.poll();
                sequenceQueue.remove(reservation);
                reservation.start();

            } else {
                chargerClient.returnToStart();
            }
        }
    }

    public void relocate(Reservation canceledReservation, LocalDateTime now) {
        Queue<Reservation> relocationQueue = new LinkedList<>();

        List<Charger> chargers = canceledReservation.getCharger().getParkingLot().getChargers();

        boolean isFound = false;
        Iterator<Reservation> iterator = sequenceQueue.iterator();
        while (iterator.hasNext()) {
            Reservation reservation = iterator.next();
            Queue<Reservation> reservations = reservationQueueMap.get(reservation.getCharger().getId());

            if (isFound) {
                reservations.remove(reservation);
                relocationQueue.add(reservation);
                iterator.remove();
            }

            if (reservation.equals(canceledReservation)) {
                isFound = true;
                reservations.remove(reservation);
                iterator.remove();
            }
        }

        // 충전기 예약시간 초기화
        for (Charger charger : chargers) {
            Deque<Reservation> reservations = reservationQueueMap.get(charger.getId());

            Reservation lastReservation = reservations.peekLast();

            if (lastReservation != null) {
                charger.updateLastReservedTime(lastReservation.getStartTime());
            } else {
                charger.updateLastReservedTime(now);
            }
        }

        // 대기큐 -> 작업큐로 예약 옮기기
        Iterator<Reservation> relocationIterator = relocationQueue.iterator();
        while (relocationIterator.hasNext()) {
            Reservation reservation = relocationIterator.next();

            // 가장 빨리 끝나는 충전기 찾기
            Charger charger = chargers.stream()
                    .filter(c -> c.getStatus().isAvailable())
                    .min((c1, c2) -> c1.getLastReservedTime().isBefore(c2.getLastReservedTime()) ? -1 : 1)
                    .orElseThrow(() -> new BaseException(ErrorCode.NO_AVAILABLE_CHARGER));

            Queue<Reservation> reservations = reservationQueueMap.get(charger.getId());
            reservations.add(reservation);
            sequenceQueue.add(reservation);

            // 바뀐 시간 업데이트하기, 충전기와 예약 둘다
            long gapMinutes = Duration.between(reservation.getStartTime(), reservation.getEndTime()).toMinutes();

            LocalDateTime startTime = charger.getLastReservedTime();
            LocalDateTime endTime = startTime.plusMinutes(gapMinutes);

            charger.updateLastReservedTime(endTime);
            reservation.updateTime(startTime, endTime);

            relocationIterator.remove();
        }

    }

    public void deleteAllReservations() {
        for (Long l : reservationQueueMap.keySet()) {
            reservationQueueMap.get(l).clear();
        }
    }

}

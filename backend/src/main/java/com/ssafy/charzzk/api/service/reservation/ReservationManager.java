package com.ssafy.charzzk.api.service.reservation;

import com.ssafy.charzzk.core.apiclient.ChargerClient;
import com.ssafy.charzzk.core.apiclient.request.ChargerCancelRequest;
import com.ssafy.charzzk.core.apiclient.request.ChargerCommandRequest;
import com.ssafy.charzzk.core.exception.BaseException;
import com.ssafy.charzzk.core.exception.ErrorCode;
import com.ssafy.charzzk.domain.charger.Charger;
import com.ssafy.charzzk.domain.reservation.Reservation;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.stereotype.Component;

import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentLinkedQueue;

@RequiredArgsConstructor
@Component
@Getter
@Slf4j
public class ReservationManager {

    private Map<Long, Queue<Reservation>> reservationQueueMap = new ConcurrentHashMap<>();
    private Queue<Reservation> sequenceQueue = new ConcurrentLinkedQueue<>();
    private Queue<Reservation> relocationQueue = new ConcurrentLinkedQueue<>();

    private final ChargerClient chargerClient;

    public void init(List<Charger> chargers) {
        for (Charger charger : chargers) {
            reservationQueueMap.put(charger.getId(), new LinkedList<>());
        }
    }

    public void createReservation(Reservation reservation) {
        reservationQueueMap.get(reservation.getCharger().getId()).add(reservation);
        sequenceQueue.add(reservation);
    }

    public void confirmReservation(Reservation reservation) {

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

    public Reservation cancelReservation(Reservation reservation) {

        Queue<Reservation> reservations = reservationQueueMap.get(reservation.getCharger().getId());

        if (reservation.isWaiting()) { // 대기중이면 취소
            sequenceQueue.remove(reservation);
        } else if(reservation.isCharging()){ // 충전중이면 중지 명령 내리기
            chargerClient.cancel(ChargerCancelRequest.of(reservation));
            reservation.getCharger().stopCharge();
        } else {
            throw new BaseException(ErrorCode.CANNOT_CANCEL_RESERVATION);
        }

        reservations.remove(reservation);
        reservation.cancel();

        // 예약 재분배

        executeNextReservation(reservation.getCharger());

        return reservation;
    }

    public void timeout(Reservation reservation) {
        Queue<Reservation> reservations = reservationQueueMap.get(reservation.getCharger().getId());
        reservations.remove(reservation);
        sequenceQueue.remove(reservation);

        // 예약 재분배
    }


    public void executeNextReservation(Charger charger) {
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

    public void deleteAllReservations() {
        for (Long l : reservationQueueMap.keySet()) {
            reservationQueueMap.get(l).clear();
        }
    }

}

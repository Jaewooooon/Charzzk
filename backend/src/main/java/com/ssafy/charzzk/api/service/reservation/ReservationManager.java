package com.ssafy.charzzk.api.service.reservation;

import com.ssafy.charzzk.core.apiclient.ChargerClient;
import com.ssafy.charzzk.core.apiclient.request.ChargerCancelRequest;
import com.ssafy.charzzk.core.apiclient.request.ChargerCommandRequest;
import com.ssafy.charzzk.core.apiclient.response.ChargerCommandResponse;
import com.ssafy.charzzk.core.exception.BaseException;
import com.ssafy.charzzk.core.exception.ErrorCode;
import com.ssafy.charzzk.domain.charger.Charger;
import com.ssafy.charzzk.domain.reservation.Reservation;
import com.ssafy.charzzk.domain.reservation.ReservationStatus;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.stereotype.Component;

import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.concurrent.ConcurrentHashMap;

@RequiredArgsConstructor
@Component
@Getter
@Slf4j
public class ReservationManager {

    private Map<Long, Queue<Reservation>> reservationQueueMap = new ConcurrentHashMap<>();

    private final ChargerClient chargerClient;

    public void init(List<Charger> chargers) {
        for (Charger charger : chargers) {
            reservationQueueMap.put(charger.getId(), new LinkedList<>());
        }
    }

    public Reservation createReservation(Reservation reservation) {

        Queue<Reservation> reservations = reservationQueueMap.get(reservation.getCharger().getId());
        reservations.add(reservation);

        printReservationQueueMap();

        return reservation;
    }

    public Reservation confirmReservation(Reservation reservation) {

        Queue<Reservation> reservations = reservationQueueMap.get(reservation.getCharger().getId());

        reservations.stream().forEach(r -> {
            if (r.getId().equals(reservation.getId())) {
                r.confirm();
                reservation.confirm();
            }
        });

        /**
         * 충전기에 해당하는 큐에서 이 예약을 찾아서 상태를 WAITING으로 바꾸기
         * 해당 예약이 1순위이고 해당 충전기가 이용가능하면 이동 명령
         * FastAPI서버 요청 보내기
         */
        if (!reservations.peek().equals(reservation)) {
            log.info("충전기의 큐에서 이 예약이 1순위가 아닙니다.");
        }

        if (!reservation.getCharger().getStatus().isWaiting()) {
            log.info("충전기가 사용중입니다.");
        }

        if (reservations.peek().equals(reservation) && reservation.getCharger().getStatus().isWaiting()) {
            // FastAPI 서버 요청
            ChargerCommandRequest chargerCommandRequest = ChargerCommandRequest.of(reservation);
            ChargerCommandResponse response = chargerClient.command(chargerCommandRequest);

            // 성공하면 충전기의 상태 바꾸기
            if (response.getStatus().equals("success")) {
                reservations.poll();
                Charger charger = reservation.getCharger();
                charger.startCharge();
                reservation.getCar().startCharge();
                reservation.start();
            }
        }

        return reservation;
    }

    public void printReservationQueueMap() {
        for (Map.Entry<Long, Queue<Reservation>> entry : reservationQueueMap.entrySet()) {
            Long chargerId = entry.getKey();
            Queue<Reservation> reservations = entry.getValue();

            System.out.println("Charger ID: " + chargerId);
            System.out.println("Reservations: ");

            for (Reservation reservation : reservations) {
                System.out.println(" - " + reservation.toString()); // 예약의 toString() 메서드가 적절하게 구현되어 있어야 합니다.
            }
        }
    }

    public Reservation cancelReservation(Reservation reservation) {
        /**
         * 예약의 상태 확인
         * 충전중인지 / 대기중인지
         * 1. 충전중이면 임베디드 서버에 중지 명령 내리기
         * 1-1. 큐에 확정된 다음 예약이 있다면 명령 내리기
         * 2. 대기중이면 큐에서 삭제
         * 예약 상태 바꾸기
         */

        Queue<Reservation> reservations = reservationQueueMap.get(reservation.getCharger().getId());

        if (reservation.getStatus().isWaiting()) { // 대기중이면 취소
            reservations.remove(reservation);
        } else { // 충전중이면 중지 명령 내리기
            ChargerCancelRequest request = ChargerCancelRequest.of(reservation);
            ChargerCommandResponse response = chargerClient.cancel(request);

            if (response.getStatus().equals("success")) {
                Charger charger = reservation.getCharger();
                charger.stopCharge();
            }
        }

        // 큐의 peek()를 확인해서 WAITING상태면 fastAPI 서버에 명령 내리기
        Reservation nextReservation = reservations.peek();

        if (nextReservation != null && nextReservation.getStatus().equals(ReservationStatus.WAITING)) {
            ChargerCommandRequest chargerCommandRequest = ChargerCommandRequest.of(reservation);
            ChargerCommandResponse response = chargerClient.command(chargerCommandRequest);

            if (response.getStatus().equals("success")) {
                reservations.poll();
                Charger charger = nextReservation.getCharger();
                charger.startCharge();
                reservation.getCar().startCharge();
                nextReservation.start();
            }
        }

        reservation.cancel();

        return reservation;
    }

    public void timeout(Reservation reservation) {
        Queue<Reservation> reservations = reservationQueueMap.get(reservation.getCharger().getId());
        reservations.remove(reservation);
    }


    public void executeNextReservation(Charger charger) {
        Queue<Reservation> reservations = reservationQueueMap.get(charger.getId());

        Reservation reservation = reservations.peek();

        if (!reservations.peek().equals(reservation)) {
            log.info("충전기의 큐에서 이 예약이 1순위가 아닙니다.");
        }

        if (!reservation.getCharger().getStatus().isWaiting()) {
            log.info("충전기가 사용중입니다.");
        }

        if (reservations.peek().getStatus().equals(ReservationStatus.WAITING)) {
            // FastAPI 서버 요청
            ChargerCommandRequest chargerCommandRequest = ChargerCommandRequest.of(reservation);
            ChargerCommandResponse response = chargerClient.command(chargerCommandRequest);

            // 성공하면 충전기의 상태 바꾸기
            if (response.getStatus().equals("success")) {
                reservations.poll();
                reservation.getCharger().startCharge();
                reservation.getCar().startCharge();
                reservation.start();
            }
        } else { // 다음 예약을 수행할 수 없으면 기본위치로 이동
            chargerClient.returnToStart();
        }
    }

    public void deleteAllReservations() {
        for (Long l : reservationQueueMap.keySet()) {
            reservationQueueMap.get(l).clear();
        }
    }


}

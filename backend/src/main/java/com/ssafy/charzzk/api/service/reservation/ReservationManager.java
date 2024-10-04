package com.ssafy.charzzk.api.service.reservation;

import com.ssafy.charzzk.core.apiclient.ChargerClient;
import com.ssafy.charzzk.core.apiclient.request.ChargerCommandRequest;
import com.ssafy.charzzk.core.apiclient.response.ChargerCommandResponse;
import com.ssafy.charzzk.domain.charger.Charger;
import com.ssafy.charzzk.domain.reservation.Reservation;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Component;

import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.concurrent.ConcurrentHashMap;

@RequiredArgsConstructor
@Component
@Getter
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
            }
        });

        /**
         * 충전기에 해당하는 큐에서 이 예약을 찾아서 상태를 WAITING으로 바꾸기
         * 해당 예약이 1순위이고 해당 충전기가 이용가능하면 이동 명령
         * FastAPI서버 요청 보내기
         */
        if (reservations.peek().equals(reservation) && reservation.getCharger().getStatus().isWaiting()) {
            // FastAPI 서버 요청
            ChargerCommandRequest chargerCommandRequest = ChargerCommandRequest.of(reservation);
            ChargerCommandResponse response = chargerClient.command(chargerCommandRequest);

            // 성공하면 충전기의 상태 바꾸기
            if (response.getStatus().equals("success")) {
                reservations.poll();
                Charger charger = reservation.getCharger();
                charger.startCharge();
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

}

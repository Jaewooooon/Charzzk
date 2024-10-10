package com.ssafy.charzzk.api.service.reservation;

import com.ssafy.charzzk.IntegrationTestSupport;
import com.ssafy.charzzk.core.apiclient.ChargerClient;
import com.ssafy.charzzk.core.exception.BaseException;
import com.ssafy.charzzk.core.exception.ErrorCode;
import com.ssafy.charzzk.domain.car.Car;
import com.ssafy.charzzk.domain.charger.Charger;
import com.ssafy.charzzk.domain.charger.ChargerStatus;
import com.ssafy.charzzk.domain.parkinglot.Location;
import com.ssafy.charzzk.domain.parkinglot.ParkingSpot;
import com.ssafy.charzzk.domain.reservation.Reservation;
import com.ssafy.charzzk.domain.reservation.ReservationStatus;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.test.mock.mockito.MockBean;
import org.springframework.test.context.ActiveProfiles;
import org.springframework.transaction.annotation.Transactional;

import java.time.LocalDateTime;
import java.util.List;
import java.util.Queue;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.BDDMockito.given;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.verify;

@Transactional
@ActiveProfiles("test")
class ReservationManagerTest extends IntegrationTestSupport {

    @Autowired
    private ReservationManager reservationManager;

    @MockBean
    private ChargerClient chargerClient;

    @AfterEach
    public void tearDown() {
        reservationManager.getReservationQueueMap().clear();
        reservationManager.getSequenceQueue().clear();
    }

    @DisplayName("예약을 만들면 해당 충전기 예약 큐에 쌓이고 순서큐에 저장된다.")
    @Test
    public void createReservation() {
        // given
        Charger charger1 = Charger.builder()
                .id(1L)
                .status(ChargerStatus.WAITING)
                .build();

        Charger charger2 = Charger.builder()
                .id(2L)
                .status(ChargerStatus.WAITING)
                .build();

        ParkingSpot parkingSpot = ParkingSpot.builder()
                .location(Location.builder().latitude(37.123456).longitude(127.123456).build())
                .build();

        Reservation reservation = Reservation.builder()
                .charger(charger1)
                .parkingSpot(parkingSpot)
                .startTime(LocalDateTime.of(2024, 1, 1, 0, 0))
                .endTime(LocalDateTime.of(2024, 1, 1, 1, 0))
                .build();

        reservationManager.init(List.of(charger1, charger2));

        Queue<Reservation> reservations = reservationManager.getReservationQueueMap().get(charger1.getId());
        Queue<Reservation> sequenceQueue = reservationManager.getSequenceQueue();

        // when
        reservationManager.createReservation(reservation);

        // then
        assertThat(reservations.size()).isEqualTo(1);
        assertThat(reservations.peek()).isEqualTo(reservation);
        assertThat(sequenceQueue.size()).isEqualTo(1);
        assertThat(sequenceQueue.peek()).isEqualTo(reservation);
    }

    @DisplayName("예약 확정에 성공할 때 예약이 1순위이고 충전기가 이용가능하면 충전을 시작한다.")
    @Test
    public void confirmReservation() {
        // given
        Car car = Car.builder()
                .isCharging(false)
                .build();

        Charger charger1 = Charger.builder()
                .id(1L)
                .status(ChargerStatus.WAITING)
                .build();

        Charger charger2 = Charger.builder()
                .id(2L)
                .status(ChargerStatus.WAITING)
                .build();

        ParkingSpot parkingSpot = ParkingSpot.builder()
                .location(Location.builder().latitude(37.123456).longitude(127.123456).build())
                .build();

        Reservation reservation = Reservation.builder()
                .car(car)
                .charger(charger1)
                .parkingSpot(parkingSpot)
                .startTime(LocalDateTime.of(2024, 1, 1, 0, 0))
                .endTime(LocalDateTime.of(2024, 1, 1, 1, 0))
                .build();

        reservationManager.init(List.of(charger1, charger2));

        Queue<Reservation> reservations = reservationManager.getReservationQueueMap().get(charger1.getId());
        Queue<Reservation> sequenceQueue = reservationManager.getSequenceQueue();

        reservations.add(reservation);
        sequenceQueue.add(reservation);

        // when
        reservationManager.confirmReservation(reservation);

        // then
        assertThat(reservation.getStatus()).isEqualTo(ReservationStatus.CHARGING);
        assertThat(reservations.size()).isEqualTo(0);
        assertThat(sequenceQueue.size()).isEqualTo(0);
        verify(chargerClient).command(any());
    }

    @DisplayName("예약을 확정할 때 큐에 예약이 없으면 예외가 발생한다.")
    @Test
    public void confirmReservationWithoutReservation() {
        // given
        Car car = Car.builder()
                .isCharging(false)
                .build();

        Charger charger1 = Charger.builder()
                .id(1L)
                .status(ChargerStatus.WAITING)
                .build();

        Charger charger2 = Charger.builder()
                .id(2L)
                .status(ChargerStatus.WAITING)
                .build();

        ParkingSpot parkingSpot = ParkingSpot.builder()
                .location(Location.builder().latitude(37.123456).longitude(127.123456).build())
                .build();

        Reservation reservation = Reservation.builder()
                .car(car)
                .charger(charger1)
                .parkingSpot(parkingSpot)
                .startTime(LocalDateTime.of(2024, 1, 1, 0, 0))
                .endTime(LocalDateTime.of(2024, 1, 1, 1, 0))
                .build();

        reservationManager.init(List.of(charger1, charger2));

        // when, then
        assertThatThrownBy(() -> reservationManager.confirmReservation(reservation))
                .isInstanceOf(BaseException.class)
                .hasMessage(ErrorCode.RESERVATION_NOT_FOUND.getMessage());
    }


    @DisplayName("예약 확정에 성공헀는데 예약이 1순위가 아니면 충전을 시작하지 않는다.")
    @Test
    public void confirmReservationWithNot1stReservation() {
        // given
        Car car = Car.builder()
                .isCharging(false)
                .build();

        Charger charger1 = Charger.builder()
                .id(1L)
                .status(ChargerStatus.WAITING)
                .build();

        Charger charger2 = Charger.builder()
                .id(2L)
                .status(ChargerStatus.WAITING)
                .build();

        ParkingSpot parkingSpot = ParkingSpot.builder()
                .location(Location.builder().latitude(37.123456).longitude(127.123456).build())
                .build();

        Reservation reservation1 = Reservation.builder()
                .id(1L)
                .car(car)
                .charger(charger1)
                .parkingSpot(parkingSpot)
                .startTime(LocalDateTime.of(2024, 1, 1, 0, 0))
                .endTime(LocalDateTime.of(2024, 1, 1, 1, 0))
                .status(ReservationStatus.WAITING)
                .build();

        Reservation reservation2 = Reservation.builder()
                .id(2L)
                .car(car)
                .charger(charger1)
                .parkingSpot(parkingSpot)
                .startTime(LocalDateTime.of(2024, 1, 1, 0, 0))
                .endTime(LocalDateTime.of(2024, 1, 1, 1, 0))
                .status(ReservationStatus.PENDING)
                .build();

        reservationManager.init(List.of(charger1, charger2));

        Queue<Reservation> reservations = reservationManager.getReservationQueueMap().get(charger1.getId());
        Queue<Reservation> sequenceQueue = reservationManager.getSequenceQueue();

        reservations.addAll(List.of(reservation1, reservation2));
        sequenceQueue.addAll(List.of(reservation1, reservation2));

        // when
        reservationManager.confirmReservation(reservation2);

        // then
        assertThat(reservation2.getStatus()).isEqualTo(ReservationStatus.WAITING);
        assertThat(reservations.size()).isEqualTo(2);
        verify(chargerClient, never()).command(any());
    }

    @DisplayName("예약 확정에 성공헀는데 충전기가 이용중이면 충전을 시작하지 않는다.")
    @Test
    public void confirmReservationWithNot1stReservationAndChargerNotAvailable() {
        // given
        Car car = Car.builder()
                .isCharging(false)
                .build();

        Charger charger1 = Charger.builder()
                .id(1L)
                .status(ChargerStatus.CAR_CHARGING)
                .build();

        Charger charger2 = Charger.builder()
                .id(2L)
                .status(ChargerStatus.CAR_CHARGING)
                .build();

        ParkingSpot parkingSpot = ParkingSpot.builder()
                .location(Location.builder().latitude(37.123456).longitude(127.123456).build())
                .build();

        Reservation reservation1 = Reservation.builder()
                .id(1L)
                .car(car)
                .charger(charger1)
                .parkingSpot(parkingSpot)
                .startTime(LocalDateTime.of(2024, 1, 1, 0, 0))
                .endTime(LocalDateTime.of(2024, 1, 1, 1, 0))
                .status(ReservationStatus.WAITING)
                .build();

        Reservation reservation2 = Reservation.builder()
                .id(2L)
                .car(car)
                .charger(charger1)
                .parkingSpot(parkingSpot)
                .startTime(LocalDateTime.of(2024, 1, 1, 0, 0))
                .endTime(LocalDateTime.of(2024, 1, 1, 1, 0))
                .status(ReservationStatus.PENDING)
                .build();

        reservationManager.init(List.of(charger1, charger2));

        Queue<Reservation> reservations = reservationManager.getReservationQueueMap().get(charger1.getId());
        Queue<Reservation> sequenceQueue = reservationManager.getSequenceQueue();

        reservations.addAll(List.of(reservation1, reservation2));
        sequenceQueue.addAll(List.of(reservation1, reservation2));

        // when
        reservationManager.confirmReservation(reservation1);

        // then
        assertThat(reservation1.getStatus()).isEqualTo(ReservationStatus.WAITING);
        assertThat(reservations.size()).isEqualTo(2);
        verify(chargerClient, never()).command(any());
    }

    @DisplayName("대기중인 예약을 취소하면 대기중인 예약은 큐에서 삭제한다.")
    @Test
    void cancelReservation() {
        // given
        Car car = Car.builder()
                .isCharging(false)
                .build();

        Charger charger1 = Charger.builder()
                .id(1L)
                .status(ChargerStatus.CAR_CHARGING)
                .build();

        Charger charger2 = Charger.builder()
                .id(2L)
                .status(ChargerStatus.CAR_CHARGING)
                .build();

        ParkingSpot parkingSpot = ParkingSpot.builder()
                .location(Location.builder().latitude(37.123456).longitude(127.123456).build())
                .build();

        Reservation reservation1 = Reservation.builder()
                .id(1L)
                .car(car)
                .charger(charger1)
                .parkingSpot(parkingSpot)
                .startTime(LocalDateTime.of(2024, 1, 1, 0, 0))
                .endTime(LocalDateTime.of(2024, 1, 1, 1, 0))
                .status(ReservationStatus.WAITING)
                .build();

        reservationManager.init(List.of(charger1, charger2));

        Queue<Reservation> reservations = reservationManager.getReservationQueueMap().get(charger1.getId());
        Queue<Reservation> sequenceQueue = reservationManager.getSequenceQueue();

        reservations.add(reservation1);
        sequenceQueue.add(reservation1);

        // when
        reservationManager.cancelReservation(reservation1);

        // then
        assertFalse(reservations.contains(reservation1));
        assertFalse(sequenceQueue.contains(reservation1));
        assertThat(reservation1.getStatus()).isEqualTo(ReservationStatus.CANCELED);
    }

    @DisplayName("충전중인 예약을 취소하면 대기중인 예약은 큐에서 삭제한다.")
    @Test
    void cancelReservationWithChargingStatus() {
        // given
        Car car = Car.builder()
                .isCharging(false)
                .build();

        Charger charger1 = Charger.builder()
                .id(1L)
                .status(ChargerStatus.CAR_CHARGING)
                .build();

        Charger charger2 = Charger.builder()
                .id(2L)
                .status(ChargerStatus.CAR_CHARGING)
                .build();

        ParkingSpot parkingSpot = ParkingSpot.builder()
                .location(Location.builder().latitude(37.123456).longitude(127.123456).build())
                .build();

        Reservation reservation1 = Reservation.builder()
                .id(1L)
                .car(car)
                .charger(charger1)
                .parkingSpot(parkingSpot)
                .startTime(LocalDateTime.of(2024, 1, 1, 0, 0))
                .endTime(LocalDateTime.of(2024, 1, 1, 1, 0))
                .status(ReservationStatus.CHARGING)
                .build();

        reservationManager.init(List.of(charger1, charger2));

        Queue<Reservation> reservations = reservationManager.getReservationQueueMap().get(charger1.getId());
        Queue<Reservation> sequenceQueue = reservationManager.getSequenceQueue();

        reservations.add(reservation1);
        sequenceQueue.add(reservation1);


        // when
        reservationManager.cancelReservation(reservation1);

        // then
        assertFalse(reservations.contains(reservation1));
        assertTrue(sequenceQueue.contains(reservation1));
        assertThat(reservation1.getStatus()).isEqualTo(ReservationStatus.CANCELED);
        assertThat(reservation1.getCharger().getStatus()).isEqualTo(ChargerStatus.WAITING);
    }

    @DisplayName("끝난 예약을 취소하면 예외가 발생한다.")
    @Test
    void cancelReservationWithDoneReservation() {
        // given
        Car car = Car.builder()
                .isCharging(false)
                .build();

        Charger charger1 = Charger.builder()
                .id(1L)
                .status(ChargerStatus.CAR_CHARGING)
                .build();

        Charger charger2 = Charger.builder()
                .id(2L)
                .status(ChargerStatus.CAR_CHARGING)
                .build();

        ParkingSpot parkingSpot = ParkingSpot.builder()
                .location(Location.builder().latitude(37.123456).longitude(127.123456).build())
                .build();

        Reservation reservation1 = Reservation.builder()
                .id(1L)
                .car(car)
                .charger(charger1)
                .parkingSpot(parkingSpot)
                .startTime(LocalDateTime.of(2024, 1, 1, 0, 0))
                .endTime(LocalDateTime.of(2024, 1, 1, 1, 0))
                .status(ReservationStatus.DONE)
                .build();

        reservationManager.init(List.of(charger1, charger2));

        // when
        assertThatThrownBy(() -> reservationManager.cancelReservation(reservation1))
                .isInstanceOf(BaseException.class)
                .hasMessage(ErrorCode.CANNOT_CANCEL_RESERVATION.getMessage());
    }


    @DisplayName("예약 확정시간이 지나면 큐에서 삭제한다.")
    @Test
    void timeout() {
        // given
        Charger charger1 = Charger.builder()
                .id(1L)
                .status(ChargerStatus.WAITING)
                .build();

        Charger charger2 = Charger.builder()
                .id(2L)
                .status(ChargerStatus.WAITING)
                .build();

        ParkingSpot parkingSpot = ParkingSpot.builder()
                .location(Location.builder().latitude(37.123456).longitude(127.123456).build())
                .build();

        Reservation reservation = Reservation.builder()
                .charger(charger1)
                .parkingSpot(parkingSpot)
                .startTime(LocalDateTime.of(2024, 1, 1, 0, 0))
                .endTime(LocalDateTime.of(2024, 1, 1, 1, 0))
                .build();

        reservationManager.init(List.of(charger1, charger2));

        Queue<Reservation> reservations = reservationManager.getReservationQueueMap().get(charger1.getId());
        Queue<Reservation> sequenceQueue = reservationManager.getSequenceQueue();

        reservations.add(reservation);
        sequenceQueue.add(reservation);

        // when
        reservationManager.timeout(reservation);

        // then
        assertThat(reservations.size()).isEqualTo(0);
        assertThat(sequenceQueue.size()).isEqualTo(0);
    }

    @DisplayName("충전기의 다음 예약을 실행한다.")
    @Test
    void executeNextReservation() {
        // given
        Car car = Car.builder()
                .isCharging(false)
                .build();

        Charger charger1 = Charger.builder()
                .id(1L)
                .status(ChargerStatus.WAITING)
                .build();

        Charger charger2 = Charger.builder()
                .id(2L)
                .status(ChargerStatus.WAITING)
                .build();

        ParkingSpot parkingSpot = ParkingSpot.builder()
                .location(Location.builder().latitude(37.123456).longitude(127.123456).build())
                .build();

        Reservation reservation = Reservation.builder()
                .status(ReservationStatus.WAITING)
                .car(car)
                .charger(charger1)
                .parkingSpot(parkingSpot)
                .startTime(LocalDateTime.of(2024, 1, 1, 0, 0))
                .endTime(LocalDateTime.of(2024, 1, 1, 1, 0))
                .build();

        reservationManager.init(List.of(charger1, charger2));

        Queue<Reservation> reservations = reservationManager.getReservationQueueMap().get(charger1.getId());
        Queue<Reservation> sequenceQueue = reservationManager.getSequenceQueue();

        reservations.add(reservation);
        sequenceQueue.add(reservation);

        // when
        reservationManager.executeNextReservation(charger1);
        
        // then
        assertThat(reservation.getStatus()).isEqualTo(ReservationStatus.CHARGING);
        assertThat(reservations.size()).isEqualTo(0);
        assertThat(sequenceQueue.size()).isEqualTo(0);
        verify(chargerClient).command(any());
    }

    @DisplayName("충전기의 다음 예약이 확정되지 않았으면 시작 지점으로 가는 명령을 실행한다.")
    @Test
    void executeNextReservationWithConfirmedReservation() {
        // given
        Car car = Car.builder()
                .isCharging(false)
                .build();

        Charger charger1 = Charger.builder()
                .id(1L)
                .status(ChargerStatus.WAITING)
                .build();

        Charger charger2 = Charger.builder()
                .id(2L)
                .status(ChargerStatus.WAITING)
                .build();

        ParkingSpot parkingSpot = ParkingSpot.builder()
                .location(Location.builder().latitude(37.123456).longitude(127.123456).build())
                .build();

        Reservation reservation = Reservation.builder()
                .status(ReservationStatus.PENDING)
                .car(car)
                .charger(charger1)
                .parkingSpot(parkingSpot)
                .startTime(LocalDateTime.of(2024, 1, 1, 0, 0))
                .endTime(LocalDateTime.of(2024, 1, 1, 1, 0))
                .build();

        reservationManager.init(List.of(charger1, charger2));

        Queue<Reservation> reservations = reservationManager.getReservationQueueMap().get(charger1.getId());
        Queue<Reservation> sequenceQueue = reservationManager.getSequenceQueue();

        reservations.add(reservation);
        sequenceQueue.add(reservation);

        // when
        reservationManager.executeNextReservation(charger1);

        // then
        assertThat(reservation.getStatus()).isEqualTo(ReservationStatus.PENDING);
        assertThat(reservations.size()).isEqualTo(1);
        assertThat(sequenceQueue.size()).isEqualTo(1);
        verify(chargerClient).returnToStart();
    }

    @DisplayName("충전기의 다음 예약이 없으면 시작지점으로 가는 명령을 내린다.")
    @Test
    void executeNextReservationWithoutNextReservation() {
        // given
        Charger charger1 = Charger.builder()
                .id(1L)
                .status(ChargerStatus.WAITING)
                .build();

        Charger charger2 = Charger.builder()
                .id(2L)
                .status(ChargerStatus.WAITING)
                .build();

        reservationManager.init(List.of(charger1, charger2));



        // when
        reservationManager.executeNextReservation(charger1);

        // then
        verify(chargerClient).returnToStart();
    }

}

package com.ssafy.charzzk.domain.reservation;

import com.ssafy.charzzk.domain.car.Car;
import com.ssafy.charzzk.domain.car.CarType;
import com.ssafy.charzzk.domain.charger.Charger;
import com.ssafy.charzzk.domain.charger.ChargerStatus;
import com.ssafy.charzzk.domain.parkinglot.Location;
import com.ssafy.charzzk.domain.parkinglot.ParkingLot;
import com.ssafy.charzzk.domain.parkinglot.ParkingSpot;
import com.ssafy.charzzk.domain.user.User;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

import java.time.LocalDateTime;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class ReservationTest {

    @DisplayName("예약 생성")
    @Test
    public void create() {
        User user = User.builder()
                .username("user@google.com")
                .nickname("user")
                .build();

        CarType carType = CarType.builder()
                .name("테스트 차종")
                .build();

        Car car = Car.builder()
                .user(user)
                .carType(carType)
                .number("12다1234")
                .build();

        Location location = Location.builder()
                .latitude(37.123456)
                .longitude(127.123456)
                .build();

        ParkingLot parkingLot = ParkingLot.builder()
                .name("테스트 주차장")
                .location(location)
                .build();

        ParkingSpot parkingSpot = ParkingSpot.builder()
                .parkingLot(parkingLot)
                .name("1")
                .location(location)
                .build();

        Charger charger = Charger.builder()
                .parkingLot(parkingLot)
                .serialNumber("1")
                .status(ChargerStatus.WAITING)
                .lastReservedTime(LocalDateTime.of(2024, 1, 1, 2, 0))
                .build();

        parkingLot.getChargers().add(charger);

        Reservation reservation = Reservation.create(car, parkingSpot, charger, LocalDateTime.of(2024, 1, 1, 2, 0), LocalDateTime.of(2024, 1, 1, 3, 0));

        assertThat(reservation).isNotNull();
    }

    @DisplayName("예약의 상태가 WAITING, CHARGING이면 True를 반환하고 아니면 False를 반환한다")
    @Test
    public void inUsing() {
        Reservation reservation1 = Reservation.builder()
                .status(ReservationStatus.WAITING)
                .build();

        Reservation reservation2 = Reservation.builder()
                .status(ReservationStatus.CHARGING)
                .build();

        assertTrue(reservation1.inUsing());
        assertTrue(reservation2.inUsing());
    }

    @DisplayName("예약의 아이디가 같으면 같은 객체다")
    @Test
    public void equals() {
        Reservation reservation1 = Reservation.builder()
                .id(1L)
                .build();

        Reservation reservation2 = Reservation.builder()
                .id(1L)
                .build();

        Reservation reservation3 = Reservation.builder()
                .id(2L)
                .build();

        CarType carType = CarType.builder()
                .name("테스트 차종")
                .build();

        assertTrue(reservation1.equals(reservation1));
        assertTrue(reservation1.equals(reservation2));
        assertFalse(reservation1.equals(reservation3));
        assertFalse(reservation1.equals(carType));
        assertFalse(reservation2.equals(null));
        assertThat(reservation1.hashCode()).isEqualTo(reservation2.hashCode());
    }

    @DisplayName("예약을 확정하면 상태를 WAITING으로 변경한다")
    @Test
    public void confirm() {
        Reservation reservation = Reservation.builder()
                .status(ReservationStatus.PENDING)
                .build();

        reservation.confirm();

        assertThat(reservation.getStatus()).isEqualTo(ReservationStatus.WAITING);
    }

    @DisplayName("예약이 대기 상태인지 확인한다.")
    @Test
    public void isWaiting() {
        Reservation reservation1 = Reservation.builder()
                .status(ReservationStatus.WAITING)
                .build();
        Reservation reservation2 = Reservation.builder()
                .status(ReservationStatus.PENDING)
                .build();

        assertTrue(reservation1.isWaiting());
        assertFalse(reservation2.isWaiting());
    }

    @DisplayName("예약이 충전 상태인지 확인한다.")
    @Test
    public void isCharging() {
        Reservation reservation1 = Reservation.builder()
                .status(ReservationStatus.CHARGING)
                .build();
        Reservation reservation2 = Reservation.builder()
                .status(ReservationStatus.PENDING)
                .build();

        assertTrue(reservation1.isCharging());
        assertFalse(reservation2.isCharging());
    }

    @DisplayName("예약을 취소하면 상태를 CANCELED으로 변경한다")
    @Test
    public void cancel() {
        Reservation reservation = Reservation.builder()
                .status(ReservationStatus.PENDING)
                .build();

        reservation.cancel();

        assertThat(reservation.getStatus()).isEqualTo(ReservationStatus.CANCELED);
    }

    @DisplayName("충전을 시작하면 상태를 CHARGING으로 변경한다")
    @Test
    public void start() {
        Car car = Car.builder()
                .number("12다1234")
                .build();

        Charger charger = Charger.builder()
                .status(ChargerStatus.WAITING)
                .build();

        Reservation reservation = Reservation.builder()
                .car(car)
                .charger(charger)
                .status(ReservationStatus.WAITING)
                .build();

        reservation.start();

        assertThat(reservation.getStatus()).isEqualTo(ReservationStatus.CHARGING);
    }

    @DisplayName("충전이 완료되면 상태를 DONE으로 변경한다")
    @Test
    public void chargeComplete() {
        User user = User.builder()
                .username("user@google.com")
                .nickname("user")
                .build();

        CarType carType = CarType.builder()
                .name("테스트 차종")
                .build();

        Car car = Car.builder()
                .user(user)
                .carType(carType)
                .number("12다1234")
                .build();

        Location location = Location.builder()
                .latitude(37.123456)
                .longitude(127.123456)
                .build();

        ParkingLot parkingLot = ParkingLot.builder()
                .name("테스트 주차장")
                .location(location)
                .build();

        ParkingSpot parkingSpot = ParkingSpot.builder()
                .parkingLot(parkingLot)
                .name("1")
                .location(location)
                .build();

        Charger charger = Charger.builder()
                .parkingLot(parkingLot)
                .serialNumber("1")
                .status(ChargerStatus.WAITING)
                .lastReservedTime(LocalDateTime.of(2024, 1, 1, 2, 0))
                .build();

        Reservation reservation = Reservation.builder()
                .car(car)
                .charger(charger)
                .status(ReservationStatus.CHARGING)
                .build();

        reservation.chargeComplete();

        assertThat(reservation.getStatus()).isEqualTo(ReservationStatus.DONE);
        assertThat(charger.getStatus()).isEqualTo(ChargerStatus.WAITING);
        assertFalse(car.isCharging());
    }

    @DisplayName("예약 시간을 변경한다.")
    @Test
    public void updateTime() {
        Reservation reservation = Reservation.builder()
                .startTime(LocalDateTime.of(2024, 1, 1, 2, 0))
                .endTime(LocalDateTime.of(2024, 1, 1, 3, 0))
                .build();

        reservation.updateTime(LocalDateTime.of(2024, 1, 1, 3, 0), LocalDateTime.of(2024, 1, 1, 4, 0));

        assertThat(reservation.getStartTime()).isEqualTo(LocalDateTime.of(2024, 1, 1, 3, 0));
        assertThat(reservation.getEndTime()).isEqualTo(LocalDateTime.of(2024, 1, 1, 4, 0));
    }

}

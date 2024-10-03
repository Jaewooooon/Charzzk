package com.ssafy.charzzk.api.service.reservation;

import com.ssafy.charzzk.IntegrationTestSupport;
import com.ssafy.charzzk.core.exception.BaseException;
import com.ssafy.charzzk.core.exception.ErrorCode;
import com.ssafy.charzzk.core.util.ChargeTimeCalculator;
import com.ssafy.charzzk.domain.car.Car;
import com.ssafy.charzzk.domain.car.CarRepository;
import com.ssafy.charzzk.domain.car.CarType;
import com.ssafy.charzzk.domain.charger.Charger;
import com.ssafy.charzzk.domain.charger.ChargerRepository;
import com.ssafy.charzzk.domain.charger.ChargerStatus;
import com.ssafy.charzzk.domain.parkinglot.Location;
import com.ssafy.charzzk.domain.parkinglot.ParkingLot;
import com.ssafy.charzzk.domain.parkinglot.ParkingLotRepository;
import com.ssafy.charzzk.domain.parkinglot.ParkingSpot;
import com.ssafy.charzzk.domain.reservation.Reservation;
import com.ssafy.charzzk.domain.reservation.ReservationCacheRepository;
import com.ssafy.charzzk.domain.reservation.ReservationConst;
import com.ssafy.charzzk.domain.reservation.ReservationRepository;
import com.ssafy.charzzk.domain.reservation.ReservationStatus;
import com.ssafy.charzzk.domain.user.User;
import com.ssafy.charzzk.domain.user.UserRepository;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.test.context.ActiveProfiles;
import org.springframework.transaction.annotation.Transactional;
import org.testcontainers.shaded.org.checkerframework.checker.units.qual.A;

import java.time.LocalDateTime;
import java.util.List;
import java.util.Queue;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.verify;

@Transactional
@ActiveProfiles("test")
class ReservationManagerTest extends IntegrationTestSupport {

    @Autowired
    private ReservationManager reservationManager;

    @Autowired
    private ReservationRepository reservationRepository;

    @Autowired
    private ChargerRepository chargerRepository;

    @Autowired
    private ParkingLotRepository parkingLotRepository;

    @Autowired
    private CarRepository carRepository;

    @DisplayName("예약을 만들면 해당 충전기의 큐에 쌓인다.")
    @Test
    public void createReservation() {
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

        Charger charger1 = Charger.builder()
                .parkingLot(parkingLot)
                .serialNumber("1")
                .status(ChargerStatus.WAITING)
                .lastReservedTime(LocalDateTime.of(2024, 1, 1, 1, 0))
                .build();

        Charger charger2 = Charger.builder()
                .parkingLot(parkingLot)
                .serialNumber("2")
                .status(ChargerStatus.WAITING)
                .lastReservedTime(LocalDateTime.of(2024, 1, 1, 1, 0))
                .build();

        Charger charger3 = Charger.builder()
                .parkingLot(parkingLot)
                .serialNumber("3")
                .status(ChargerStatus.WAITING)
                .lastReservedTime(LocalDateTime.of(2024, 1, 1, 1, 0))
                .build();

        parkingLot.getChargers().add(charger1);
        parkingLot.getChargers().add(charger2);
        parkingLot.getChargers().add(charger3);

        Reservation reservation = Reservation.builder()
                .car(car)
                .parkingSpot(parkingSpot)
                .charger(charger1)
                .startTime(LocalDateTime.of(2024, 1, 1, 0, 0))
                .endTime(LocalDateTime.of(2024, 1, 1, 1, 0))
                .status(ReservationStatus.PENDING)
                .build();

        parkingLotRepository.save(parkingLot);
        chargerRepository.saveAll(List.of(charger1, charger2, charger3));

        reservationManager.init(chargerRepository.findAll());

        // when
        reservationManager.createReservation(reservation);

        // then
        Queue<Reservation> reservations = reservationManager.getReservationQueueMap().get(charger1.getId());
        assertThat(reservations.size()).isEqualTo(1);
        assertThat(reservations.peek()).isEqualTo(reservation);
    }

    @DisplayName("예약을 확정하면 큐에서 해당 예약의 상태를 PENDING에서 WAITING으로 변경한다.")
    @Test
    public void confirmReservation() {
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

        Charger charger1 = Charger.builder()
                .parkingLot(parkingLot)
                .serialNumber("1")
                .status(ChargerStatus.WAITING)
                .lastReservedTime(LocalDateTime.of(2024, 1, 1, 1, 0))
                .build();

        Charger charger2 = Charger.builder()
                .parkingLot(parkingLot)
                .serialNumber("2")
                .status(ChargerStatus.WAITING)
                .lastReservedTime(LocalDateTime.of(2024, 1, 1, 1, 0))
                .build();

        Charger charger3 = Charger.builder()
                .parkingLot(parkingLot)
                .serialNumber("3")
                .status(ChargerStatus.WAITING)
                .lastReservedTime(LocalDateTime.of(2024, 1, 1, 1, 0))
                .build();

        parkingLot.getChargers().add(charger1);
        parkingLot.getChargers().add(charger2);
        parkingLot.getChargers().add(charger3);

        Reservation reservation = Reservation.builder()
                .car(car)
                .parkingSpot(parkingSpot)
                .charger(charger1)
                .startTime(LocalDateTime.of(2024, 1, 1, 0, 0))
                .endTime(LocalDateTime.of(2024, 1, 1, 1, 0))
                .status(ReservationStatus.PENDING)
                .build();

        parkingLotRepository.save(parkingLot);
        chargerRepository.saveAll(List.of(charger1, charger2, charger3));
        carRepository.save(car);
        reservationRepository.save(reservation);

        reservationManager.init(chargerRepository.findAll());

        Queue<Reservation> reservations = reservationManager.getReservationQueueMap().get(reservation.getCharger().getId());
        reservations.add(reservation);

        // when
        reservationManager.confirmReservation(reservation);

        // then
        assertThat(reservation.getStatus()).isEqualTo(ReservationStatus.WAITING);
    }
}

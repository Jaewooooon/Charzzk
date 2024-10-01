package com.ssafy.charzzk.api.service.reservation;

import com.ssafy.charzzk.IntegrationTestSupport;
import com.ssafy.charzzk.api.service.reservation.response.ReservationResponse;
import com.ssafy.charzzk.core.exception.BaseException;
import com.ssafy.charzzk.core.exception.ErrorCode;
import com.ssafy.charzzk.core.util.ChargeTimeCalculator;
import com.ssafy.charzzk.domain.car.Car;
import com.ssafy.charzzk.domain.car.CarRepository;
import com.ssafy.charzzk.domain.car.CarType;
import com.ssafy.charzzk.domain.charger.Charger;
import com.ssafy.charzzk.domain.charger.ChargerStatus;
import com.ssafy.charzzk.domain.parkinglot.Location;
import com.ssafy.charzzk.domain.parkinglot.ParkingLot;
import com.ssafy.charzzk.domain.parkinglot.ParkingLotRepository;
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
import org.springframework.boot.test.mock.mockito.MockBean;
import org.springframework.test.context.ActiveProfiles;
import org.springframework.transaction.annotation.Transactional;

import java.time.LocalDateTime;
import java.util.List;

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
    private CarRepository carRepository;

    @Autowired
    private ParkingLotRepository parkingLotRepository;

    @Autowired
    private UserRepository userRepository;

    @Autowired
    private ReservationRepository reservationRepository;

    @Autowired
    private ReservationCacheRepository reservationCacheRepository;

    @AfterEach
    public void tearDown() {
        reservationCacheRepository.deleteAllReservations();
    }

    @DisplayName("예약을 생성할 때 충전기들 중 가장 빠른 시간 선택하되 현재 시간과 마지막 예약시간을 비교해 현재시간이 나중이면 현재시간을 선택한다.")
    @Test
    public void createReservationWithLastReservedTimeIsAfterThenNow() {
        // given
        User user = User.builder()
                .username("user@google.com")
                .nickname("user")
                .build();

        CarType carType = CarType.builder()
                .name("아이오닉")
                .build();

        Car car = Car.builder()
                .user(user)
                .carType(carType)
                .number("12다1234")
                .battery(40)
                .build();

        Location location = Location.builder()
                .latitude(37.123456)
                .longitude(127.123456)
                .build();

        ParkingLot parkingLot = ParkingLot.builder()
                .name("테스트 주차장")
                .location(location)
                .build();

        LocalDateTime C1lastReservedTime = LocalDateTime.of(2024, 1, 1, 0, 0);
        Charger charger1 = Charger.builder()
                .parkingLot(parkingLot)
                .serialNumber("1")
                .status(ChargerStatus.WAITING)
                .lastReservedTime(C1lastReservedTime)
                .build();

        Charger charger2 = Charger.builder()
                .parkingLot(parkingLot)
                .serialNumber("2")
                .status(ChargerStatus.WAITING)
                .lastReservedTime(LocalDateTime.of(2024, 1, 1, 2, 0))
                .build();

        parkingLot.getChargers().add(charger1);
        parkingLot.getChargers().add(charger2);

        carRepository.save(car);
        parkingLotRepository.save(parkingLot);

        int time = 0;
        boolean fullCharge = true;
        LocalDateTime now = LocalDateTime.of(2024, 1, 1, 1, 0);

        int duration = ChargeTimeCalculator.calculate(car, fullCharge, time);

        // when
        Reservation reservation = reservationManager.createReservation(parkingLot, car, time, fullCharge, now);

        // then
        assertThat(charger1.getLastReservedTime()).isEqualTo(now);
        assertThat(reservation).isNotNull()
                .extracting("car", "charger", "startTime", "endTime", "status")
                .containsExactly(car, charger1, now, now.plusMinutes(duration), ReservationStatus.PENDING);

    }

    @DisplayName("예약을 생성할 때 충전기들 중 가장 빠른 시간 선택하되 현재 시간과 마지막 예약시간을 비교해 현재시간이 나중이면 현재시간을 선택한다.")
    @Test
    public void createReservationWithNowIsAfterThenLastReservedTime() {

        // given
        User user = User.builder()
                .username("user@google.com")
                .nickname("user")
                .build();

        CarType carType = CarType.builder()
                .name("아이오닉")
                .build();

        Car car = Car.builder()
                .user(user)
                .carType(carType)
                .number("12다1234")
                .battery(40)
                .build();

        Location location = Location.builder()
                .latitude(37.123456)
                .longitude(127.123456)
                .build();

        ParkingLot parkingLot = ParkingLot.builder()
                .name("테스트 주차장")
                .location(location)
                .build();

        LocalDateTime C1lastReservedTime = LocalDateTime.of(2024, 1, 1, 1, 0);
        Charger charger1 = Charger.builder()
                .parkingLot(parkingLot)
                .serialNumber("1")
                .status(ChargerStatus.WAITING)
                .lastReservedTime(C1lastReservedTime)
                .build();

        Charger charger2 = Charger.builder()
                .parkingLot(parkingLot)
                .serialNumber("2")
                .status(ChargerStatus.WAITING)
                .lastReservedTime(LocalDateTime.of(2024, 1, 1, 2, 0))
                .build();

        parkingLot.getChargers().add(charger1);
        parkingLot.getChargers().add(charger2);

        carRepository.save(car);
        parkingLotRepository.save(parkingLot);

        int time = 0;
        boolean fullCharge = true;
        LocalDateTime now = LocalDateTime.of(2024, 1, 1, 0, 0);

        int duration = ChargeTimeCalculator.calculate(car, fullCharge, time);

        // when
        Reservation reservation = reservationManager.createReservation(parkingLot, car, time, fullCharge, now);

        // then
        assertThat(charger1.getLastReservedTime()).isEqualTo(C1lastReservedTime);
        assertThat(reservation).isNotNull()
                .extracting("car", "charger", "startTime", "endTime", "status")
                .containsExactly(car, charger1, C1lastReservedTime, C1lastReservedTime.plusMinutes(duration), ReservationStatus.PENDING);

    }


    @DisplayName("예약 확정을 할 때 유예시간 이내면 예약을 확정한다.")
    @Test
    public void confirmReservationInTime() {
        // given
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

        Charger charger1 = Charger.builder()
                .parkingLot(parkingLot)
                .serialNumber("1")
                .status(ChargerStatus.WAITING)
                .lastReservedTime(LocalDateTime.of(2024, 1, 1, 2, 0))
                .build();

        Charger charger2 = Charger.builder()
                .parkingLot(parkingLot)
                .serialNumber("2")
                .status(ChargerStatus.WAITING)
                .lastReservedTime(LocalDateTime.of(2024, 1, 1, 2, 0))
                .build();

        parkingLot.getChargers().add(charger1);
        parkingLot.getChargers().add(charger2);

        userRepository.save(user);
        carRepository.save(car);
        parkingLotRepository.save(parkingLot);

        Reservation reservation = Reservation.builder()
                .car(car)
                .charger(charger1)
                .startTime(LocalDateTime.of(2024, 1, 1, 0, 0))
                .endTime(LocalDateTime.of(2024, 1, 1, 1, 0))
                .status(ReservationStatus.PENDING)
                .build();

        reservationRepository.save(reservation);
        reservationCacheRepository.createReservationGracePeriod(reservation.getId(), charger1.getId(), ReservationConst.gracePeriod);

        reservationManager.init();

        // when
        Reservation confirmedReservation = reservationManager.confirmReservation(user, reservation.getId());

        // then
        assertThat(confirmedReservation).isNotNull();
        assertThat(confirmedReservation.getStatus()).isEqualTo(ReservationStatus.WAITING);
    }


    @DisplayName("예약 확정을 할 때 유예시간 이후면 예외를 발생시킨다.")
    @Test
    public void confirmReservationNotInTime() throws InterruptedException {
        // given
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

        Charger charger1 = Charger.builder()
                .parkingLot(parkingLot)
                .serialNumber("1")
                .status(ChargerStatus.WAITING)
                .lastReservedTime(LocalDateTime.of(2024, 1, 1, 2, 0))
                .build();

        Charger charger2 = Charger.builder()
                .parkingLot(parkingLot)
                .serialNumber("2")
                .status(ChargerStatus.WAITING)
                .lastReservedTime(LocalDateTime.of(2024, 1, 1, 2, 0))
                .build();

        parkingLot.getChargers().add(charger1);
        parkingLot.getChargers().add(charger2);

        userRepository.save(user);
        carRepository.save(car);
        parkingLotRepository.save(parkingLot);

        Reservation reservation = Reservation.builder()
                .car(car)
                .charger(charger1)
                .startTime(LocalDateTime.of(2024, 1, 1, 0, 0))
                .endTime(LocalDateTime.of(2024, 1, 1, 1, 0))
                .status(ReservationStatus.PENDING)
                .build();

        reservationRepository.save(reservation);
        reservationCacheRepository.createReservationGracePeriod(reservation.getId(), charger1.getId(), 1);

        reservationManager.init();

        Thread.sleep(1000);

        // when
        assertThatThrownBy(() -> reservationManager.confirmReservation(user, reservation.getId()))
                .isInstanceOf(BaseException.class)
                .hasMessageContaining(ErrorCode.RESERVATION_CONFIRM_TIMEOUT.getMessage());

    }

    @DisplayName("예약 확정을 할 때 예약이 없으면 예외가 발생한다.")
    @Test
    public void confirmReservationWithReservationNotFound() {
        // given
        User user = User.builder()
                .username("user@google.com")
                .nickname("user")
                .build();

        userRepository.save(user);

        // when, then
        assertThatThrownBy(() -> reservationManager.confirmReservation(user, 1L))
                .isInstanceOf(BaseException.class)
                .hasMessageContaining(ErrorCode.RESERVATION_NOT_FOUND.getMessage());

    }

    @DisplayName("예약 확정을 유저와 예약의 차 주인이 일치하지 않으면 예외가 발생한다.")
    @Test
    public void confirmReservationWithNotMyCar() throws InterruptedException {
        // given
        User user1 = User.builder()
                .username("user1@google.com")
                .nickname("user1")
                .build();

        User user2 = User.builder()
                .username("user2@google.com")
                .nickname("user2")
                .build();

        CarType carType = CarType.builder()
                .name("테스트 차종")
                .build();

        Car car = Car.builder()
                .user(user2)
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

        Charger charger1 = Charger.builder()
                .parkingLot(parkingLot)
                .serialNumber("1")
                .status(ChargerStatus.WAITING)
                .lastReservedTime(LocalDateTime.of(2024, 1, 1, 2, 0))
                .build();

        Charger charger2 = Charger.builder()
                .parkingLot(parkingLot)
                .serialNumber("2")
                .status(ChargerStatus.WAITING)
                .lastReservedTime(LocalDateTime.of(2024, 1, 1, 2, 0))
                .build();

        parkingLot.getChargers().add(charger1);
        parkingLot.getChargers().add(charger2);

        userRepository.saveAll(List.of(user1, user2));
        carRepository.save(car);
        parkingLotRepository.save(parkingLot);

        Reservation reservation = Reservation.builder()
                .id(1L)
                .car(car)
                .charger(charger1)
                .startTime(LocalDateTime.of(2024, 1, 1, 0, 0))
                .endTime(LocalDateTime.of(2024, 1, 1, 1, 0))
                .status(ReservationStatus.PENDING)
                .build();

        reservationRepository.save(reservation);

        // when, then
        assertThatThrownBy(() -> reservationManager.confirmReservation(user1, reservation.getId()))
                .isInstanceOf(BaseException.class)
                .hasMessageContaining(ErrorCode.FORBIDDEN.getMessage());
    }

}

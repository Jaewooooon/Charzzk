package com.ssafy.charzzk.api.service.reservation;

import com.ssafy.charzzk.IntegrationTestSupport;
import com.ssafy.charzzk.api.service.reservation.request.ReservationConfirmServiceRequest;
import com.ssafy.charzzk.api.service.reservation.request.ReservationServiceRequest;
import com.ssafy.charzzk.api.service.reservation.response.ReservationResponse;
import com.ssafy.charzzk.core.exception.BaseException;
import com.ssafy.charzzk.core.exception.ErrorCode;
import com.ssafy.charzzk.core.util.ChargeTimeCalculator;
import com.ssafy.charzzk.domain.car.Car;
import com.ssafy.charzzk.domain.car.CarRepository;
import com.ssafy.charzzk.domain.car.CarType;
import com.ssafy.charzzk.domain.charger.Charger;
import com.ssafy.charzzk.domain.charger.ChargerStatus;
import com.ssafy.charzzk.domain.parkinglot.*;
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
import static org.mockito.ArgumentMatchers.anyInt;
import static org.mockito.BDDMockito.given;
import static org.mockito.Mockito.verify;

@Transactional
@ActiveProfiles("test")
class ReservationServiceTest extends IntegrationTestSupport {

    @Autowired
    private ReservationService reservationService;

    @Autowired
    private ReservationRepository reservationRepository;

    @Autowired
    private UserRepository userRepository;

    @Autowired
    private ParkingLotRepository parkingLotRepository;

    @Autowired
    private CarRepository carRepository;

    @Autowired
    private ParkingSpotRepository parkingSpotRepository;

    @Autowired
    private ReservationCacheRepository reservationCacheRepository;

    @MockBean
    private ReservationManager reservationManager;

    @AfterEach
    void tearDown() {
        reservationCacheRepository.deleteAllReservations();
    }

    @DisplayName("예약 아이디로 예약을 조회한다.")
    @Test
    public void getReservation() {
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

        Reservation reservation = Reservation.builder()
                .car(car)
                .parkingSpot(parkingSpot)
                .charger(charger)
                .startTime(LocalDateTime.of(2024, 1, 1, 0, 0))
                .endTime(LocalDateTime.of(2024, 1, 1, 1, 0))
                .status(ReservationStatus.PENDING)
                .build();

        userRepository.save(user);
        carRepository.save(car);
        parkingLotRepository.save(parkingLot);
        reservationRepository.save(reservation);

        // when
        ReservationResponse response = reservationService.getReservation(reservation.getId());

        // then
        assertThat(response).isNotNull()
                .extracting("id", "car.id", "startTime", "endTime", "status")
                .containsExactly(reservation.getId(), car.getId(), reservation.getStartTime(), reservation.getEndTime(), ReservationStatus.PENDING.name());
    }

    @DisplayName("예약을 성공적으로 생성하면 예약을 반환한다 (예약의 충전기1을 선택하는 경우).")
    @Test
    public void createReservation1() {
        // given
        LocalDateTime time = LocalDateTime.of(2024, 1, 1, 0, 0);

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

        LocalDateTime charger1LastReservedTime = LocalDateTime.of(2024, 1, 1, 1, 0);
        Charger charger1 = Charger.builder()
                .parkingLot(parkingLot)
                .serialNumber("1")
                .status(ChargerStatus.WAITING)
                .lastReservedTime(charger1LastReservedTime)
                .build();

        Charger charger2 = Charger.builder()
                .parkingLot(parkingLot)
                .serialNumber("2")
                .status(ChargerStatus.WAITING)
                .lastReservedTime(LocalDateTime.of(2024, 1, 1, 2, 0))
                .build();

        parkingLot.getChargers().add(charger1);
        parkingLot.getChargers().add(charger2);

        ParkingSpot parkingSpot = ParkingSpot.builder()
                .parkingLot(parkingLot)
                .name("1")
                .location(location)
                .build();

        userRepository.save(user);
        carRepository.save(car);
        parkingLotRepository.save(parkingLot);
        parkingSpotRepository.save(parkingSpot);

        ReservationServiceRequest request = ReservationServiceRequest.builder()
                .parkingSpotId(parkingSpot.getId())
                .carId(car.getId())
                .parkingLotId(parkingLot.getId())
                .fullCharge(true)
                .time(0)
                .build();

        int duration = ChargeTimeCalculator.calculate(car, request.isFullCharge(), request.getTime());

        // when
        Reservation reservation = reservationService.create(user, request, time);

        // then
        assertThat(reservation).isNotNull()
                .extracting("car", "parkingSpot", "charger", "startTime", "endTime", "status")
                .containsExactly(car, parkingSpot, charger1, charger1LastReservedTime, charger1LastReservedTime.plusMinutes(duration), ReservationStatus.PENDING);

        verify(reservationManager).createReservation(any(Reservation.class));
    }

    @DisplayName("예약을 성공적으로 생성하면 예약을 반환한다 (예약의 충전기2를 선택하는 경우).")
    @Test
    public void createReservation2() {
        // given
        LocalDateTime time = LocalDateTime.of(2024, 1, 1, 0, 0);

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

        LocalDateTime charger2LastReservedTime = LocalDateTime.of(2024, 1, 1, 1, 0);
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
                .lastReservedTime(charger2LastReservedTime)
                .build();

        parkingLot.getChargers().add(charger1);
        parkingLot.getChargers().add(charger2);

        ParkingSpot parkingSpot = ParkingSpot.builder()
                .parkingLot(parkingLot)
                .name("1")
                .location(location)
                .build();

        userRepository.save(user);
        carRepository.save(car);
        parkingLotRepository.save(parkingLot);
        parkingSpotRepository.save(parkingSpot);

        ReservationServiceRequest request = ReservationServiceRequest.builder()
                .parkingSpotId(parkingSpot.getId())
                .carId(car.getId())
                .parkingLotId(parkingLot.getId())
                .fullCharge(true)
                .time(0)
                .build();

        int duration = ChargeTimeCalculator.calculate(car, request.isFullCharge(), request.getTime());

        // when
        Reservation reservation = reservationService.create(user, request, time);

        // then
        assertThat(reservation).isNotNull()
                .extracting("car", "parkingSpot", "charger", "startTime", "endTime", "status")
                .containsExactly(car, parkingSpot, charger2, charger2LastReservedTime, charger2LastReservedTime.plusMinutes(duration), ReservationStatus.PENDING);

        verify(reservationManager).createReservation(any(Reservation.class));
    }

    @DisplayName("예약을 성공적으로 생성하면 예약을 반환한다 (예약의 충전기2를 선택하는 경우).")
    @Test
    public void createReservation3() {
        // given
        LocalDateTime time = LocalDateTime.of(2025, 1, 1, 0, 0);

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

        LocalDateTime charger2LastReservedTime = LocalDateTime.of(2024, 1, 1, 1, 0);
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
                .lastReservedTime(charger2LastReservedTime)
                .build();

        parkingLot.getChargers().add(charger1);
        parkingLot.getChargers().add(charger2);

        ParkingSpot parkingSpot = ParkingSpot.builder()
                .parkingLot(parkingLot)
                .name("1")
                .location(location)
                .build();

        userRepository.save(user);
        carRepository.save(car);
        parkingLotRepository.save(parkingLot);
        parkingSpotRepository.save(parkingSpot);

        ReservationServiceRequest request = ReservationServiceRequest.builder()
                .parkingSpotId(parkingSpot.getId())
                .carId(car.getId())
                .parkingLotId(parkingLot.getId())
                .fullCharge(true)
                .time(0)
                .build();

        int duration = ChargeTimeCalculator.calculate(car, request.isFullCharge(), request.getTime());

        // when
        Reservation reservation = reservationService.create(user, request, time);

        // then
        assertThat(reservation).isNotNull()
                .extracting("car", "parkingSpot", "charger", "startTime", "endTime", "status")
                .containsExactly(car, parkingSpot, charger2, time, time.plusMinutes(duration), ReservationStatus.PENDING);

        verify(reservationManager).createReservation(any(Reservation.class));
    }

    @DisplayName("예약을 생성할 때 차량을 찾을 수 없으면 예외가 발생한다.")
    @Test
    public void createReservationWithInvalidCar() {
        // given
        LocalDateTime time = LocalDateTime.of(2024, 1, 1, 0, 0);

        User user = User.builder()
                .username("user@google.com")
                .nickname("user")
                .build();

        ReservationServiceRequest request = ReservationServiceRequest.builder()
                .carId(1L)
                .parkingLotId(1L)
                .fullCharge(true)
                .time(0)
                .build();

        // when, then
        assertThatThrownBy(() -> reservationService.create(user, request, time))
                .isInstanceOf(BaseException.class)
                .hasMessage(ErrorCode.CAR_NOT_FOUND.getMessage());
    }

    @DisplayName("예약을 생성할 때 차량이 사용자의 차량이 아니면 예외가 발생한다.")
    @Test
    public void createReservationWithNotMyCar() {
        // given
        LocalDateTime time = LocalDateTime.of(2024, 1, 1, 0, 0);

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
                .user(user1)
                .carType(carType)
                .number("12다1234")
                .build();


        userRepository.saveAll(List.of(user1, user2));
        carRepository.save(car);


        ReservationServiceRequest request = ReservationServiceRequest.builder()
                .carId(car.getId())
                .parkingLotId(1L)
                .fullCharge(true)
                .time(0)
                .build();

        // when, then
        assertThatThrownBy(() -> reservationService.create(user2, request, time))
                .isInstanceOf(BaseException.class)
                .hasMessage(ErrorCode.FORBIDDEN.getMessage());
    }

    @DisplayName("예약을 생성할 주차장을 찾을 수 없으면 예외가 발생한다.")
    @Test
    public void createReservationWithInvalidParkingLot() {
        // given
        LocalDateTime time = LocalDateTime.of(2024, 1, 1, 0, 0);

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

        userRepository.save(user);
        carRepository.save(car);

        ReservationServiceRequest request = ReservationServiceRequest.builder()
                .carId(car.getId())
                .parkingLotId(1L)
                .fullCharge(true)
                .time(0)
                .build();

        // when, then
        assertThatThrownBy(() -> reservationService.create(user, request, time))
                .isInstanceOf(BaseException.class)
                .hasMessage(ErrorCode.PARKING_LOT_NOT_FOUND.getMessage());
    }


    @DisplayName("예약을 생성할 때 주차칸을 찾을 수 없으면 예외가 발생한다.")
    @Test
    public void createReservationWithInvalidParkingSpot() {
        // given
        LocalDateTime time = LocalDateTime.of(2024, 1, 1, 0, 0);

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

        LocalDateTime charger1LastReservedTime = LocalDateTime.of(2024, 1, 1, 1, 0);
        Charger charger1 = Charger.builder()
                .parkingLot(parkingLot)
                .serialNumber("1")
                .status(ChargerStatus.WAITING)
                .lastReservedTime(charger1LastReservedTime)
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

        ReservationServiceRequest request = ReservationServiceRequest.builder()
                .parkingSpotId(1L)
                .carId(car.getId())
                .parkingLotId(parkingLot.getId())
                .fullCharge(true)
                .time(0)
                .build();

        assertThatThrownBy(() -> reservationService.create(user, request, time))
                .isInstanceOf(BaseException.class)
                .hasMessage(ErrorCode.PARKING_SPOT_NOT_FOUND.getMessage());
    }

    @DisplayName("주차장에 이용 가능한 충전기가 없으면 예외가 발생한다.")
    @Test
    public void createReservationWithoutAvailableChargers() {
        // given
        LocalDateTime time = LocalDateTime.of(2024, 1, 1, 0, 0);

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

        LocalDateTime charger1LastReservedTime = LocalDateTime.of(2024, 1, 1, 1, 0);
        Charger charger1 = Charger.builder()
                .parkingLot(parkingLot)
                .serialNumber("1")
                .status(ChargerStatus.ERROR)
                .lastReservedTime(charger1LastReservedTime)
                .build();

        Charger charger2 = Charger.builder()
                .parkingLot(parkingLot)
                .serialNumber("2")
                .status(ChargerStatus.ERROR)
                .lastReservedTime(LocalDateTime.of(2024, 1, 1, 2, 0))
                .build();

        parkingLot.getChargers().add(charger1);
        parkingLot.getChargers().add(charger2);

        ParkingSpot parkingSpot = ParkingSpot.builder()
                .parkingLot(parkingLot)
                .name("1")
                .location(location)
                .build();

        userRepository.save(user);
        carRepository.save(car);
        parkingLotRepository.save(parkingLot);
        parkingSpotRepository.save(parkingSpot);

        ReservationServiceRequest request = ReservationServiceRequest.builder()
                .parkingSpotId(parkingSpot.getId())
                .carId(car.getId())
                .parkingLotId(parkingLot.getId())
                .fullCharge(true)
                .time(0)
                .build();


        // when
        assertThatThrownBy(() -> reservationService.create(user, request, time))
                .isInstanceOf(BaseException.class)
                .hasMessage(ErrorCode.NO_AVAILABLE_CHARGER.getMessage());
    }

    @DisplayName("예약을 성공적으로 확정하면 예약을 반환한다.")
    @Test
    public void confirm() {
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

        ParkingSpot parkingSpot = ParkingSpot.builder()
                .parkingLot(parkingLot)
                .name("1")
                .location(location)
                .build();

        userRepository.save(user);
        carRepository.save(car);
        parkingLotRepository.save(parkingLot);

        Reservation reservation = Reservation.builder()
                .parkingSpot(parkingSpot)
                .car(car)
                .charger(charger1)
                .startTime(LocalDateTime.of(2024, 1, 1, 0, 0))
                .endTime(LocalDateTime.of(2024, 1, 1, 1, 0))
                .status(ReservationStatus.PENDING)
                .build();

        reservationRepository.save(reservation);
        reservationCacheRepository.createReservationGracePeriod(reservation.getId(), charger1.getId(), ReservationConst.gracePeriod);


        // when
        Reservation confirmReservation = reservationService.confirm(user, reservation.getId());

        // then
        assertThat(confirmReservation).isNotNull()
                .extracting("id", "status")
                .containsExactly(reservation.getId(), ReservationStatus.PENDING);
    }

    @DisplayName("유예시간이 지난 후 예약을 확정하면 예외가 발생한다.")
    @Test
    public void confirmTimeout() throws InterruptedException {
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

        ParkingSpot parkingSpot = ParkingSpot.builder()
                .parkingLot(parkingLot)
                .name("1")
                .location(location)
                .build();

        userRepository.save(user);
        carRepository.save(car);
        parkingLotRepository.save(parkingLot);

        Reservation reservation = Reservation.builder()
                .parkingSpot(parkingSpot)
                .car(car)
                .charger(charger1)
                .startTime(LocalDateTime.of(2024, 1, 1, 0, 0))
                .endTime(LocalDateTime.of(2024, 1, 1, 1, 0))
                .status(ReservationStatus.PENDING)
                .build();

        reservationRepository.save(reservation);
        reservationCacheRepository.createReservationGracePeriod(reservation.getId(), charger1.getId(), 1);

        Thread.sleep(1000);

        // when
        assertThatThrownBy(() -> reservationService.confirm(user, reservation.getId()))
                .isInstanceOf(BaseException.class)
                .hasMessage(ErrorCode.RESERVATION_CONFIRM_TIMEOUT.getMessage());
    }

    @DisplayName("예약을 확정할 때 예약을 찾을 수 없으면 예외가 발생한다.")
    @Test
    public void confirmReservationWithoutReservation() {
        // given
        LocalDateTime time = LocalDateTime.of(2024, 1, 1, 0, 0);

        User user = User.builder()
                .username("user@google.com")
                .nickname("user")
                .build();

        // when, then
        assertThatThrownBy(() -> reservationService.confirm(user, 1L))
                .isInstanceOf(BaseException.class)
                .hasMessage(ErrorCode.RESERVATION_NOT_FOUND.getMessage());
    }

    @DisplayName("예약을 확정할 때 예약의 차 주인과 내가 다르면 예외가 발생한다.")
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

        ParkingSpot parkingSpot = ParkingSpot.builder()
                .parkingLot(parkingLot)
                .name("1")
                .location(location)
                .build();

        userRepository.saveAll(List.of(user1, user2));
        carRepository.save(car);
        parkingLotRepository.save(parkingLot);

        Reservation reservation = Reservation.builder()
                .parkingSpot(parkingSpot)
                .car(car)
                .charger(charger1)
                .startTime(LocalDateTime.of(2024, 1, 1, 0, 0))
                .endTime(LocalDateTime.of(2024, 1, 1, 1, 0))
                .status(ReservationStatus.PENDING)
                .build();

        reservationRepository.save(reservation);

        // when
        assertThatThrownBy(() -> reservationService.confirm(user1, reservation.getId()))
                .isInstanceOf(BaseException.class)
                .hasMessage(ErrorCode.FORBIDDEN.getMessage());
    }

    @DisplayName("예약을 취소할 때 예약을 찾을 수 없으면 예외가 발생한다.")
    @Test
    public void cancelReservationWithoutReservation() {
        // given
        LocalDateTime time = LocalDateTime.of(2024, 1, 1, 0, 0);

        User user = User.builder()
                .username("user@google.com")
                .nickname("user")
                .build();

        // when, then
        assertThatThrownBy(() -> reservationService.cancel(user, 1L))
                .isInstanceOf(BaseException.class)
                .hasMessage(ErrorCode.RESERVATION_NOT_FOUND.getMessage());
    }

    @DisplayName("예약을 취소할 때 예약의 차 주인과 내가 다르면 예외가 발생한다.")
    @Test
    public void cancelReservationWithNotMyCar(){
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

        ParkingSpot parkingSpot = ParkingSpot.builder()
                .parkingLot(parkingLot)
                .name("1")
                .location(location)
                .build();

        userRepository.saveAll(List.of(user1, user2));
        carRepository.save(car);
        parkingLotRepository.save(parkingLot);

        Reservation reservation = Reservation.builder()
                .parkingSpot(parkingSpot)
                .car(car)
                .charger(charger1)
                .startTime(LocalDateTime.of(2024, 1, 1, 0, 0))
                .endTime(LocalDateTime.of(2024, 1, 1, 1, 0))
                .status(ReservationStatus.PENDING)
                .build();

        reservationRepository.save(reservation);

        // when
        assertThatThrownBy(() -> reservationService.cancel(user1, reservation.getId()))
                .isInstanceOf(BaseException.class)
                .hasMessage(ErrorCode.FORBIDDEN.getMessage());
    }

    @DisplayName("예약을 성공적으로 취소하면 예약을 반환한다.")
    @Test
    public void cancel() {
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

        ParkingSpot parkingSpot = ParkingSpot.builder()
                .parkingLot(parkingLot)
                .name("1")
                .location(location)
                .build();

        userRepository.save(user);
        carRepository.save(car);
        parkingLotRepository.save(parkingLot);

        Reservation reservation = Reservation.builder()
                .parkingSpot(parkingSpot)
                .car(car)
                .charger(charger1)
                .startTime(LocalDateTime.of(2024, 1, 1, 0, 0))
                .endTime(LocalDateTime.of(2024, 1, 1, 1, 0))
                .status(ReservationStatus.PENDING)
                .build();

        Reservation canceledReservation = Reservation.builder()
                .parkingSpot(parkingSpot)
                .car(car)
                .charger(charger1)
                .startTime(LocalDateTime.of(2024, 1, 1, 0, 0))
                .endTime(LocalDateTime.of(2024, 1, 1, 1, 0))
                .status(ReservationStatus.CANCELED)
                .build();

        reservationRepository.save(reservation);

        reservation.cancel();
        given(reservationManager.cancelReservation(any(Reservation.class))).willReturn(reservation);


        // when
        Reservation cancelReservation = reservationService.cancel(user, reservation.getId());

        // then
        assertThat(cancelReservation).isNotNull()
                .extracting("id", "status")
                .containsExactly(reservation.getId(), ReservationStatus.CANCELED);
        verify(reservationManager).cancelReservation(any(Reservation.class));
    }
    
    @DisplayName("예약 확정시간이 지나면 예약을 삭제한다.")
    @Test
    public void timeout() {
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

        ParkingSpot parkingSpot = ParkingSpot.builder()
                .parkingLot(parkingLot)
                .name("1")
                .location(location)
                .build();

        userRepository.save(user);
        carRepository.save(car);
        parkingLotRepository.save(parkingLot);

        Reservation reservation = Reservation.builder()
                .parkingSpot(parkingSpot)
                .car(car)
                .charger(charger1)
                .startTime(LocalDateTime.of(2024, 1, 1, 0, 0))
                .endTime(LocalDateTime.of(2024, 1, 1, 1, 0))
                .status(ReservationStatus.PENDING)
                .build();

        reservationRepository.save(reservation);

        // when
        reservationService.timeout(reservation.getId());

        // then
        assertThat(reservationRepository.findById(reservation.getId())).isEmpty();
        verify(reservationManager).timeout(any(Reservation.class));
    }

    @DisplayName("예약 확정시간이 지났을 떄 예약이 없으면 예외가 발생한다.")
    @Test
    public void timeoutWithInvalidReservationId() {
        // when, then
        assertThatThrownBy(() -> reservationService.timeout(1L))
                .isInstanceOf(BaseException.class)
                .hasMessage(ErrorCode.RESERVATION_NOT_FOUND.getMessage());
    }

    @DisplayName("예약을 조회할 때예약이 없으면 예외가 발생한다.")
    @Test
    public void getReservationWithInvalidReservationId() {
        // when, then
        assertThatThrownBy(() -> reservationService.getReservation(1L))
                .isInstanceOf(BaseException.class)
                .hasMessage(ErrorCode.RESERVATION_NOT_FOUND.getMessage());
    }

}

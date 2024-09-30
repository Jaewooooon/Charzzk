package com.ssafy.charzzk.api.service.reservation;

import com.ssafy.charzzk.IntegrationTestSupport;
import com.ssafy.charzzk.api.service.reservation.request.ReservationServiceRequest;
import com.ssafy.charzzk.api.service.reservation.response.ReservationResponse;
import com.ssafy.charzzk.core.exception.BaseException;
import com.ssafy.charzzk.core.exception.ErrorCode;
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
import com.ssafy.charzzk.domain.reservation.ReservationRepository;
import com.ssafy.charzzk.domain.reservation.ReservationStatus;
import com.ssafy.charzzk.domain.user.User;
import com.ssafy.charzzk.domain.user.UserRepository;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.mockito.Mock;
import org.springframework.beans.factory.annotation.Autowired;
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
    private ReservationCacheRepository reservationCacheRepository;

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

        Charger charger = Charger.builder()
                .parkingLot(parkingLot)
                .serialNumber("1")
                .status(ChargerStatus.WAITING)
                .lastReservedTime(LocalDateTime.of(2024, 1, 1, 2, 0))
                .build();

        parkingLot.getChargers().add(charger);

        Reservation reservation = Reservation.builder()
                .car(car)
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
                .containsExactly(reservation.getId(), car.getId(), reservation.getStartTime(), reservation.getEndTime(), reservation.getStatus().name());
    }

    @DisplayName("예약을 생성하면 예약의 아이디를 반환한다.")
    @Test
    public void createReservation() {
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

        ReservationServiceRequest request = ReservationServiceRequest.builder()
                .carId(car.getId())
                .parkingLotId(parkingLot.getId())
                .fullCharge(true)
                .time(0)
                .build();

        // when
        Long reservationId = reservationService.create(user, request, time);

        // then
        assertThat(reservationId).isNotNull();
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

}

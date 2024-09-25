package com.ssafy.charzzk.api.service.reservation;

import com.ssafy.charzzk.IntegrationTestSupport;
import com.ssafy.charzzk.api.service.response.ReservationCheckTimeResponse;
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
import com.ssafy.charzzk.domain.reservation.ReservationRepository;
import com.ssafy.charzzk.domain.user.User;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.test.context.ActiveProfiles;
import org.springframework.transaction.annotation.Transactional;

import java.time.LocalDateTime;
import java.util.List;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.junit.jupiter.api.Assertions.*;
@Transactional
@ActiveProfiles("test")
class ReservationServiceTest extends IntegrationTestSupport {

    @Autowired
    private ReservationService reservationService;

    @Autowired
    private ParkingLotRepository parkingLotRepository;

    @Autowired
    private CarRepository carRepository;

    @Autowired
    private ReservationRepository reservationRepository;

    @DisplayName("사용 가능한 충전기가 있을 때 예약 가능한 시간을 확인한다.")
    @Test
    void checkTime() {
        // given
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
                .status(ChargerStatus.WAITING)
                .build();
        Charger charger2 = Charger.builder()
                .parkingLot(parkingLot)
                .status(ChargerStatus.CAR_CHARGING)
                .build();

        parkingLot.getChargers().add(charger1);
        parkingLot.getChargers().add(charger2);

        User user = User.builder()
                .username("test@gmail.com")
                .nickname("테스터")
                .build();

        CarType carType = CarType.builder()
                .name("테스트 차종")
                .build();

        Car car = Car.builder()
                .user(user)
                .carType(carType)
                .number("12다1234")
                .battery(50)
                .build();

        parkingLotRepository.save(parkingLot);
        carRepository.save(car);

        LocalDateTime dateTime = LocalDateTime.of(2024, 1, 1, 0, 0);
        boolean fullCharge = true;
        int time = 0;

        int duration = ChargeTimeCalculator.calculate(car, fullCharge, time);

        // when
        ReservationCheckTimeResponse response = reservationService.checkTime(parkingLot.getId(), car.getId(), fullCharge, time, dateTime);

        // then
        assertThat(response).isNotNull()
                .extracting("chargerId", "isPossibleNow", "startTime", "endTime")
                .containsExactly(charger1.getId(), true, dateTime, dateTime.plusMinutes(duration));
    }

    @DisplayName("주차장을 찾을 수 없을 때 예외가 발생한다.")
    @Test
    void checkTimeWithInvalidParkingLot() {
        // given
        LocalDateTime dateTime = LocalDateTime.of(2024, 1, 1, 0, 0);
        boolean fullCharge = true;
        int time = 0;

        // when, then
        assertThatThrownBy(() -> reservationService.checkTime(1L, 1L, fullCharge, time, dateTime))
                .isInstanceOf(BaseException.class)
                .hasMessage(ErrorCode.PARKING_LOT_NOT_FOUND.getMessage());

    }


    @DisplayName("차량을 찾을 수 없을 때 예외가 발생한다.")
    @Test
    void checkTimeWithInvalidCar()  {
        // given
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
                .status(ChargerStatus.WAITING)
                .build();
        Charger charger2 = Charger.builder()
                .parkingLot(parkingLot)
                .status(ChargerStatus.WAITING)
                .build();

        parkingLot.getChargers().add(charger1);
        parkingLot.getChargers().add(charger2);

        parkingLotRepository.save(parkingLot);

        LocalDateTime dateTime = LocalDateTime.of(2024, 1, 1, 0, 0);
        boolean fullCharge = true;
        int time = 0;

        // when, then
        assertThatThrownBy(() -> reservationService.checkTime(parkingLot.getId(), 1L, fullCharge, time, dateTime))
                .isInstanceOf(BaseException.class)
                .hasMessage(ErrorCode.CAR_NOT_FOUND.getMessage());

    }

    @DisplayName("주차장에 충전기가 없을 때 예외가 발생한다.")
    @Test
    void checkTimeWithoutCharger() {
        // given
        Location location = Location.builder()
                .latitude(37.123456)
                .longitude(127.123456)
                .build();

        ParkingLot parkingLot = ParkingLot.builder()
                .name("테스트 주차장")
                .location(location)
                .build();

        parkingLotRepository.save(parkingLot);

        LocalDateTime dateTime = LocalDateTime.of(2024, 1, 1, 0, 0);
        boolean fullCharge = true;
        int time = 0;

        // when, then
        assertThatThrownBy(() -> reservationService.checkTime(parkingLot.getId(), 1L, fullCharge, time, dateTime))
                .isInstanceOf(BaseException.class)
                .hasMessage(ErrorCode.CHARGER_NOT_EXIST_IN_PARKING_LOT.getMessage());

    }


    @DisplayName("당장 이용 가능한 충전기가 없으면 예약 가능한 시간을 확인한다.")
    @Test
    void checkTimeLater() {
        // given
        LocalDateTime dateTime = LocalDateTime.of(2024, 1, 1, 0, 0);
        boolean fullCharge = true;
        int time = 0;

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
                .status(ChargerStatus.CAR_CHARGING)
                .build();
        Charger charger2 = Charger.builder()
                .parkingLot(parkingLot)
                .status(ChargerStatus.CAR_CHARGING)
                .build();

        parkingLot.getChargers().add(charger1);
        parkingLot.getChargers().add(charger2);



        User user = User.builder()
                .username("test@gmail.com")
                .nickname("테스터")
                .build();

        CarType carType = CarType.builder()
                .name("테스트 차종")
                .build();

        Car car = Car.builder()
                .user(user)
                .carType(carType)
                .number("12다1234")
                .battery(50)
                .build();

        Reservation reservation1 = Reservation.builder()
                .charger(charger1)
                .car(car)
                .startTime(dateTime.minusMinutes(10))
                .endTime(dateTime.plusMinutes(10))
                .build();

        Reservation reservation2 = Reservation.builder()
                .charger(charger2)
                .car(car)
                .startTime(dateTime.minusMinutes(10))
                .endTime(dateTime.plusMinutes(20))
                .build();

        List<Reservation> reservations = List.of(reservation1, reservation2);

        parkingLotRepository.save(parkingLot);
        carRepository.save(car);
        reservationRepository.saveAll(reservations);

        int duration = ChargeTimeCalculator.calculate(car, fullCharge, time);

        // when
        ReservationCheckTimeResponse response = reservationService.checkTime(parkingLot.getId(), car.getId(), fullCharge, time, dateTime);

        // then
        assertThat(response).isNotNull()
                .extracting("isPossibleNow", "startTime", "endTime")
                .containsExactly(false, dateTime.plusMinutes(10), dateTime.plusMinutes(10 + duration));
    }

}

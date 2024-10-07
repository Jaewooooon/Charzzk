package com.ssafy.charzzk.api.service.message;

import com.ssafy.charzzk.IntegrationTestSupport;
import com.ssafy.charzzk.core.exception.BaseException;
import com.ssafy.charzzk.core.exception.ErrorCode;
import com.ssafy.charzzk.domain.car.Car;
import com.ssafy.charzzk.domain.car.CarRepository;
import com.ssafy.charzzk.domain.car.CarType;
import com.ssafy.charzzk.domain.charger.Charger;
import com.ssafy.charzzk.domain.charger.ChargerStatus;
import com.ssafy.charzzk.domain.parkinglot.*;
import com.ssafy.charzzk.domain.reservation.Reservation;
import com.ssafy.charzzk.domain.reservation.ReservationRepository;
import com.ssafy.charzzk.domain.reservation.ReservationStatus;
import com.ssafy.charzzk.domain.user.User;
import com.ssafy.charzzk.domain.user.UserRepository;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.test.context.ActiveProfiles;
import org.springframework.transaction.annotation.Transactional;

import java.time.LocalDateTime;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.verify;

@Transactional
@ActiveProfiles("test")
class MessageServiceTest extends IntegrationTestSupport {

    @Autowired
    private MessageService messageService;

    @Autowired
    private ParkingLotRepository parkingLotRepository;

    @Autowired
    private UserRepository userRepository;

    @Autowired
    private CarRepository carRepository;

    @Autowired
    private ReservationRepository reservationRepository;

    @DisplayName("예약에 해당하는 차량의 배터리를 업데이트한다.")
    @Test
    void updateBatteryStatus() {
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
        messageService.updateBatteryStatus(reservation.getId(), 50);

        // then
        assertThat(reservation.getCar().getBattery()).isEqualTo(50);
    }

    @DisplayName("예약에 해당하는 차량의 배터리를 업데이트할 떄 예약이 없으면 예외.")
    @Test
    void updateBatteryStatusWithoutReservation() {
        // when, then
        assertThatThrownBy(() -> messageService.updateBatteryStatus(1L, 50))
                .isInstanceOf(BaseException.class)
                .hasMessage(ErrorCode.RESERVATION_NOT_FOUND.getMessage());
    }


    @DisplayName("충전을 완료할 때 해당 예약이 없으면 예외.")
    @Test
    void chargeCompleteWithoutReservation() {
        // when, then
        assertThatThrownBy(() -> messageService.updateBatteryStatus(1L, 50))
                .isInstanceOf(BaseException.class)
                .hasMessage(ErrorCode.RESERVATION_NOT_FOUND.getMessage());
    }

}

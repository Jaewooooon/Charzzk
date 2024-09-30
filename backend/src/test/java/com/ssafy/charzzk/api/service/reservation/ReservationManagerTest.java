package com.ssafy.charzzk.api.service.reservation;

import com.ssafy.charzzk.IntegrationTestSupport;
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
import com.ssafy.charzzk.domain.reservation.ReservationStatus;
import com.ssafy.charzzk.domain.user.User;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.test.mock.mockito.MockBean;
import org.springframework.test.context.ActiveProfiles;
import org.springframework.transaction.annotation.Transactional;

import java.time.LocalDateTime;

import static org.assertj.core.api.Assertions.assertThat;
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

    @MockBean
    private ReservationCacheRepository reservationCacheRepository;


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

        verify(reservationCacheRepository).createReservationGracePeriod(reservation.getId(), charger1.getId(), ReservationConst.gracePeriod);
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

        verify(reservationCacheRepository).createReservationGracePeriod(reservation.getId(), charger1.getId(), ReservationConst.gracePeriod);
    }

}

package com.ssafy.charzzk.domain.reservation;

import com.ssafy.charzzk.IntegrationTestSupport;
import com.ssafy.charzzk.domain.car.Car;
import com.ssafy.charzzk.domain.car.CarRepository;
import com.ssafy.charzzk.domain.car.CarType;
import com.ssafy.charzzk.domain.charger.Charger;
import com.ssafy.charzzk.domain.charger.ChargerStatus;
import com.ssafy.charzzk.domain.parkinglot.Location;
import com.ssafy.charzzk.domain.parkinglot.ParkingLot;
import com.ssafy.charzzk.domain.parkinglot.ParkingLotRepository;
import com.ssafy.charzzk.domain.user.User;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.test.context.ActiveProfiles;
import org.springframework.transaction.annotation.Transactional;

import java.time.LocalDateTime;
import java.util.List;
import java.util.Optional;

import static org.junit.jupiter.api.Assertions.*;

@Transactional
@ActiveProfiles("test")
class ReservationRepositoryTest extends IntegrationTestSupport {

    @Autowired
    private ReservationRepository reservationRepository;

    @Autowired
    private ParkingLotRepository parkingLotRepository;

    @Autowired
    private CarRepository carRepository;

    @DisplayName("예약 가능 시간을 확인한다")
    @Test
    void checkTime() {
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

        Charger charger = Charger.builder()
                .parkingLot(parkingLot)
                .status(ChargerStatus.CAR_CHARGING)
                .build();

        parkingLot.getChargers().add(charger);

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
                .charger(charger)
                .car(car)
                .startTime(dateTime.minusMinutes(10))
                .endTime(dateTime.plusMinutes(10))
                .build();

        Reservation reservation2 = Reservation.builder()
                .charger(charger)
                .car(car)
                .startTime(dateTime.minusMinutes(10))
                .endTime(dateTime.plusMinutes(20))
                .build();

        List<Reservation> reservations = List.of(reservation1, reservation2);

        parkingLotRepository.save(parkingLot);
        carRepository.save(car);
        reservationRepository.saveAll(reservations);

        // when
        Optional<Reservation> findReservation = reservationRepository.findFirstByChargerIdOrderByEndTimeDesc(charger.getId());

        // then
        assertTrue(findReservation.isPresent());
        assertEquals(reservation2.getId(), findReservation.get().getId());

    }
}
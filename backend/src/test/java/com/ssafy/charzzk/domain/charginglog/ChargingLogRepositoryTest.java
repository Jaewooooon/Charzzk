package com.ssafy.charzzk.domain.charginglog;

import com.ssafy.charzzk.IntegrationTestSupport;
import com.ssafy.charzzk.domain.car.Car;
import com.ssafy.charzzk.domain.car.CarRepository;
import com.ssafy.charzzk.domain.car.CarType;
import com.ssafy.charzzk.domain.car.CarTypeRepository;
import com.ssafy.charzzk.domain.charger.Charger;
import com.ssafy.charzzk.domain.charger.ChargerRepository;
import com.ssafy.charzzk.domain.parkinglot.Location;
import com.ssafy.charzzk.domain.parkinglot.ParkingLot;
import com.ssafy.charzzk.domain.parkinglot.ParkingLotRepository;
import com.ssafy.charzzk.domain.user.User;
import com.ssafy.charzzk.domain.user.UserRepository;
import org.assertj.core.groups.Tuple;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.test.context.ActiveProfiles;
import org.springframework.transaction.annotation.Transactional;

import java.time.LocalDateTime;
import java.util.List;

import static org.assertj.core.api.Assertions.assertThat;

@Transactional
@ActiveProfiles("test")
class ChargingLogRepositoryTest extends IntegrationTestSupport {

    @Autowired
    private ChargingLogRepository chargingLogRepository;

    @Autowired
    private CarRepository carRepository;

    @Autowired
    private UserRepository userRepository;

    @Autowired
    private ChargerRepository chargerRepository;

    @Autowired
    private ParkingLotRepository parkingLotRepository;

    @Autowired
    private CarTypeRepository carTypeRepository;

    @DisplayName("특정 차량의 1달 이내의 충전 기록을 조회한다.")
    @Test
    void findByCarAndTimePeriod() {
        // given
        User user = User.builder()
                .username("testuser@gmail.com")
                .nickname("테스트유저")
                .build();
        userRepository.save(user);

        ParkingLot parkingLot = ParkingLot.builder()
                .name("테스트 주차장")
                .location(Location.builder()
                        .latitude(37.5665)
                        .longitude(126.9780)
                        .build())
                .build();
        parkingLotRepository.save(parkingLot);

        Charger charger = Charger.builder()
                .serialNumber("1234ABCD")
                .parkingLot(parkingLot)
                .build();
        chargerRepository.save(charger);

        CarType carType = CarType.builder()
                .name("현대 포터2 일렉트릭")
                .image("image/hyundai")
                .build();
        carTypeRepository.save(carType);

        Car car = Car.builder()
                .user(user)
                .carType(carType)
                .number("12가1234")
                .nickname("붕붕이")
                .isCharging(false)
                .build();
        carRepository.save(car);

        ChargingLog log1 = ChargingLog.builder()
                .car(car)
                .charger(charger)
                .startTime(LocalDateTime.of(2024, 8, 21, 11, 0))
                .endTime(LocalDateTime.of(2024, 8, 21, 14, 0))
                .build();
        ChargingLog log2 = ChargingLog.builder()
                .car(car)
                .charger(charger)
                .startTime(LocalDateTime.of(2024, 9, 1, 10, 0))
                .endTime(LocalDateTime.of(2024, 9, 1, 12, 0))
                .build();
        ChargingLog log3 = ChargingLog.builder()
                .car(car)
                .charger(charger)
                .startTime(LocalDateTime.of(2024, 9, 5, 10, 0))
                .endTime(LocalDateTime.of(2024, 9, 5, 11, 0))
                .build();
        ChargingLog log4 = ChargingLog.builder()
                .car(car)
                .charger(charger)
                .startTime(LocalDateTime.of(2024, 10, 16, 10, 0))
                .endTime(LocalDateTime.of(2024, 10, 16, 11, 0))
                .build();
        chargingLogRepository.saveAll(List.of(log1, log2, log3, log4));

        LocalDateTime startTime = LocalDateTime.of(2024, 9, 1, 0, 0, 0);
        LocalDateTime endTime = LocalDateTime.of(2024, 9, 30, 23, 59, 59);

        // when
        List<ChargingLog> logs = chargingLogRepository.findByCarAndEndTime(car, startTime, endTime);

        // then
        assertThat(logs).hasSize(2)
                .extracting("startTime", "endTime")
                .containsExactlyInAnyOrder(
                        Tuple.tuple(LocalDateTime.of(2024, 9, 1, 10, 0), LocalDateTime.of(2024, 9, 1, 12, 0)),
                        Tuple.tuple(LocalDateTime.of(2024, 9, 5, 10, 0), LocalDateTime.of(2024, 9, 5, 11, 0))
                );
    }

    @Test
    @DisplayName("특정 유저의 모든 충전 기록을 조회한다.")
    void findByUser() {

        // given
        User user1 = User.builder()
                .username("testuser1@gmail.com")
                .nickname("테스트유저1")
                .build();
        userRepository.save(user1);

        User user2 = User.builder()
                .username("testuser2@gmail.com")
                .nickname("테스트유저2")
                .build();
        userRepository.save(user2);

        ParkingLot parkingLot = ParkingLot.builder()
                .name("테스트 주차장")
                .location(Location.builder()
                        .latitude(37.5665)
                        .longitude(126.9780)
                        .build())
                .build();
        parkingLotRepository.save(parkingLot);

        Charger charger = Charger.builder()
                .serialNumber("5678EFGH")
                .parkingLot(parkingLot)
                .build();
        chargerRepository.save(charger);

        CarType carType1 = CarType.builder()
                .name("현대 포터2 일렉트릭")
                .image("image/hyundai")
                .build();
        carTypeRepository.save(carType1);

        CarType carType2 = CarType.builder()
                .name("테슬라 모델3")
                .image("image/tesla")
                .build();
        carTypeRepository.save(carType2);

        Car car1 = Car.builder()
                .user(user1)
                .carType(carType1)
                .number("12가1234")
                .nickname("붕붕이")
                .isCharging(false)
                .build();
        carRepository.save(car1);

        ChargingLog log1 = ChargingLog.builder()
                .car(car1)
                .charger(charger)
                .startTime(LocalDateTime.of(2024, 8, 25, 14, 0))
                .endTime(LocalDateTime.of(2024, 8, 25, 16, 0))
                .build();
        ChargingLog log2 = ChargingLog.builder()
                .car(car1)
                .charger(charger)
                .startTime(LocalDateTime.of(2024, 9, 2, 13, 0))
                .endTime(LocalDateTime.of(2024, 9, 2, 14, 0))
                .build();
        chargingLogRepository.saveAll(List.of(log1, log2));

        Car car2 = Car.builder()
                .user(user2)
                .carType(carType2)
                .number("34나5678")
                .nickname("닝닝")
                .isCharging(false)
                .build();
        carRepository.save(car2);

        ChargingLog log3 = ChargingLog.builder()
                .car(car2)
                .charger(charger)
                .startTime(LocalDateTime.of(2024, 8, 26, 15, 0))
                .endTime(LocalDateTime.of(2024, 8, 26, 17, 0))
                .build();
        chargingLogRepository.save(log3);


        // when
        List<ChargingLog> logs = chargingLogRepository.findByUser(user1);


        // then
        assertThat(logs).hasSize(2)
                .extracting("startTime", "endTime")
                .containsExactlyInAnyOrder(
                        Tuple.tuple(LocalDateTime.of(2024, 8, 25, 14, 0), LocalDateTime.of(2024, 8, 25, 16, 0)),
                        Tuple.tuple(LocalDateTime.of(2024, 9, 2, 13, 0), LocalDateTime.of(2024, 9, 2, 14, 0))
                );
        assertThat(logs)
                .extracting("car.number", "car.carType.name")
                .containsExactlyInAnyOrder(
                        Tuple.tuple("12가1234", "현대 포터2 일렉트릭"),
                        Tuple.tuple("12가1234", "현대 포터2 일렉트릭")
                );
    }
}
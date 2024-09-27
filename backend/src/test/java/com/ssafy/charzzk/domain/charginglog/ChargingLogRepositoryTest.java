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
    public void findByCarAndTimePeriod() {
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
}
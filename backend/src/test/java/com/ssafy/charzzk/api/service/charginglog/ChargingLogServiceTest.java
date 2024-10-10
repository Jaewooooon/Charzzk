package com.ssafy.charzzk.api.service.charginglog;

import com.ssafy.charzzk.IntegrationTestSupport;
import com.ssafy.charzzk.api.service.charginglog.response.ChargingLogResponse;
import com.ssafy.charzzk.domain.car.Car;
import com.ssafy.charzzk.domain.car.CarRepository;
import com.ssafy.charzzk.domain.car.CarType;
import com.ssafy.charzzk.domain.car.CarTypeRepository;
import com.ssafy.charzzk.domain.charger.Charger;
import com.ssafy.charzzk.domain.charger.ChargerRepository;
import com.ssafy.charzzk.domain.charginglog.ChargingLog;
import com.ssafy.charzzk.domain.charginglog.ChargingLogRepository;
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

import static com.ssafy.charzzk.api.service.car.CarConst.CHARGE_AMOUNT_PER_HOUR;
import static com.ssafy.charzzk.api.service.car.CarConst.COST_PER_KWH;
import static org.assertj.core.api.Assertions.assertThat;

@Transactional
@ActiveProfiles("test")
class ChargingLogServiceTest extends IntegrationTestSupport {

    @Autowired
    private ChargingLogService chargingLogService;

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

    @DisplayName("특정 유저의 모든 충전 로그를 조회한다.")
    @Test
    void getChargingLogList() {
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
        chargingLogRepository.saveAll(List.of(log1, log2));

        // 예상 충전량 및 비용 계산
        double expectedChargeAmount1 = CHARGE_AMOUNT_PER_HOUR * 3;
        long expectedChargeCost1 = (long) (expectedChargeAmount1 * COST_PER_KWH);
        double expectedChargeAmount2 = CHARGE_AMOUNT_PER_HOUR * 2;
        long expectedChargeCost2 = (long) (expectedChargeAmount2 * COST_PER_KWH);

        // when
        List<ChargingLogResponse> chargingLogList = chargingLogService.getChargingLogList(user);

        // then
        assertThat(chargingLogList).hasSize(2)
                .extracting("chargeAmount", "chargeCost")
                .containsExactlyInAnyOrder(
                        Tuple.tuple(expectedChargeAmount1, expectedChargeCost1),
                        Tuple.tuple(expectedChargeAmount2, expectedChargeCost2)
                );

        assertThat(chargingLogList).extracting("car.nickname")
                .containsOnly("붕붕이");
    }

}

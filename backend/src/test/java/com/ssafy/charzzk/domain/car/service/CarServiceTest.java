package com.ssafy.charzzk.domain.car.service;

import com.ssafy.charzzk.IntegrationTestSupport;
import com.ssafy.charzzk.api.service.car.CarService;
import com.ssafy.charzzk.api.service.car.request.CarServiceRequest;
import com.ssafy.charzzk.api.service.car.response.CarListResponse;
import com.ssafy.charzzk.api.service.car.response.CarReservationStatusResponse;
import com.ssafy.charzzk.api.service.car.response.CarResponse;
import com.ssafy.charzzk.api.service.car.response.CarTypeResponse;
import com.ssafy.charzzk.core.exception.BaseException;
import com.ssafy.charzzk.core.exception.ErrorCode;
import com.ssafy.charzzk.domain.car.Car;
import com.ssafy.charzzk.domain.car.CarRepository;
import com.ssafy.charzzk.domain.car.CarType;
import com.ssafy.charzzk.domain.car.CarTypeRepository;
import com.ssafy.charzzk.domain.charger.Charger;
import com.ssafy.charzzk.domain.charger.ChargerRepository;
import com.ssafy.charzzk.domain.charger.ChargerStatus;
import com.ssafy.charzzk.domain.charginglog.ChargingLog;
import com.ssafy.charzzk.domain.charginglog.ChargingLogRepository;
import com.ssafy.charzzk.domain.parkinglot.*;
import com.ssafy.charzzk.domain.reservation.Reservation;
import com.ssafy.charzzk.domain.reservation.ReservationRepository;
import com.ssafy.charzzk.domain.reservation.ReservationStatus;
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
import static org.assertj.core.api.Assertions.assertThatThrownBy;

@Transactional
@ActiveProfiles("test")
class CarServiceTest extends IntegrationTestSupport {

    @Autowired
    private CarRepository carRepository;

    @Autowired
    private CarTypeRepository carTypeRepository;

    @Autowired
    private ChargingLogRepository chargingLogRepository;

    @Autowired
    private ChargerRepository chargerRepository;

    @Autowired
    private ParkingLotRepository parkingLotRepository;

    @Autowired
    private ReservationRepository reservationRepository;

    @Autowired
    private UserRepository userRepository;

    @Autowired
    private CarService carService;

    @Autowired
    private ParkingSpotRepository parkingSpotRepository;

    @DisplayName("차량을 등록하면 정상적으로 등록된다.")
    @Test
    public void createCar() {
        // given
        User user = User.builder()
                .username("test@gmail.com")
                .nickname("nickname")
                .build();

        CarType cartype = CarType.builder()
                .name("테슬라 모델 3")
                .image("abc/def")
                .build();

        carTypeRepository.save(cartype);

        CarServiceRequest request = CarServiceRequest.builder()
                .carTypeId(cartype.getId())
                .number("11가1111")
                .nickname("콩이")
                .build();

        // when
        Long carId = carService.createCar(user, request);

        // then
        assertThat(carId).isNotNull();

    }

    @DisplayName("이미 존재하는 차량번호를 등록하면 예외가 발생한다.")
    @Test
    public void createCarWithExistingName() {
        // given
        User user = User.builder()
                .username("test@gmail.com")
                .nickname("nickname")
                .build();

        CarType cartype = CarType.builder()
                .name("테슬라 모델 3")
                .image("abc/def")
                .build();

        Car car = Car.builder()
                .carType(cartype)
                .number("11가1111")
                .user(user)
                .build();

        carRepository.save(car);

        CarServiceRequest request = CarServiceRequest.builder()
                .carTypeId(car.getCarType().getId())
                .number("11가1111")
                .nickname("콩이")
                .build();

        // when
        assertThatThrownBy(() -> carService.createCar(user, request))
                .isInstanceOf(BaseException.class)
                .hasMessage(ErrorCode.CAR_NUMBER_ALREADY_EXISTS.getMessage());
    }

    @DisplayName("존재하지 않는 차종을 등록하면 예외가 발생한다.")
    @Test
    public void createCarWithNonExistentCarType() {
        // given
        User user = User.builder()
                .username("test@gmail.com")
                .nickname("nickname")
                .build();

        CarServiceRequest request = CarServiceRequest.builder()
                .carTypeId(999L) // 존재하지 않을 carTypeId 넣기
                .number("12가1111")
                .nickname("순이")
                .build();

        // when & then
        assertThatThrownBy(() -> carService.createCar(user, request))
                .isInstanceOf(BaseException.class)
                .hasMessage(ErrorCode.CAR_TYPE_NOT_FOUND.getMessage());
    }

    @DisplayName("빈 문자열로 차종 목록을 조회하면 모든 차종을 반환한다.")
    @Test
    public void getAllCarTypes() {
        // given
        CarType carType1 = CarType.builder()
                .name("테슬라 모델 3")
                .image("image1")
                .build();

        CarType carType2 = CarType.builder()
                .name("현대 아이오닉 5")
                .image("image2")
                .build();

        carTypeRepository.save(carType1);
        carTypeRepository.save(carType2);

        // when
        List<CarTypeResponse> carTypes = carService.getCarTypes("");

        // then
        assertThat(carTypes).hasSize(2);
        assertThat(carTypes).extracting("name")
                .containsExactlyInAnyOrder("테슬라 모델 3", "현대 아이오닉 5");

    }

    @DisplayName("차량을 조회하면 정상적으로 반환된다.")
    @Test
    public void getCar() {
        User user = User.builder()
                .username("test@gmail.com")
                .nickname("nickname")
                .build();

        CarType cartype = CarType.builder()
                .name("테슬라 모델 3")
                .image("abc/def")
                .build();

        Car car = Car.builder()
                .carType(cartype)
                .number("11가1111")
                .nickname("콩이")
                .user(user)
                .build();

        carRepository.save(car);

        CarResponse response = carService.getCar(car.getId());

        assertThat(response).isNotNull()
                .extracting("number", "nickname")
                .containsExactly("11가1111", "콩이");
    }

    @DisplayName("없는 차량을 조회하면 정상적으로 반환된다.")
    @Test
    public void getCarWithInvalidCarId() {
        // when, then
        assertThatThrownBy(() -> carService.getCar(1L))
                .isInstanceOf(BaseException.class)
                .hasMessage(ErrorCode.CAR_NOT_FOUND.getMessage());
    }


    @DisplayName("차종 이름으로 검색하면 해당하는 차종들만 반환한다.")
    @Test
    public void getCarTypesByName() {
        // given
        CarType carType1 = CarType.builder()
                .name("테슬라 모델 3")
                .image("image1")
                .build();

        CarType carType2 = CarType.builder()
                .name("현대 아이오닉 5")
                .image("image2")
                .build();

        carTypeRepository.save(carType1);
        carTypeRepository.save(carType2);

        // when
        List<CarTypeResponse> carTypes = carService.getCarTypes("테슬라");

        // then
        assertThat(carTypes).hasSize(1);
        assertThat(carTypes.get(0).getName()).isEqualTo("테슬라 모델 3");

    }

    @DisplayName("차량을 수정하면 정상적으로 수정된다.")
    @Test
    public void updateCar() {
        // given
        User user = User.builder()
                .username("test@gmail.com")
                .nickname("nickname")
                .build();

        CarType cartype = CarType.builder()
                .name("테슬라 모델 3")
                .image("abc/def")
                .build();

        carTypeRepository.save(cartype);

        Car car = Car.builder()
                .carType(cartype)
                .number("11가1111")
                .nickname("콩이")
                .user(user)
                .build();

        carRepository.save(car);

        CarServiceRequest request = CarServiceRequest.builder()
                .carTypeId(cartype.getId())
                .number("22나2222")
                .nickname("새로운 콩이")
                .build();

        // when
        carService.updateCar(car.getId(), user, request);

        // then
        Car updatedCar = carRepository.findById(car.getId()).get();

        assertThat(updatedCar.getNumber()).isEqualTo("22나2222");
        assertThat(updatedCar.getNickname()).isEqualTo("새로운 콩이");

    }

    @DisplayName("없는 차를 수정하면 예외가 발생한다.")
    @Test
    public void updateCarWithInvalidCarId() {
        User user = User.builder()
                .username("test@gmail.com")
                .nickname("nickname")
                .build();

        CarType cartype = CarType.builder()
                .name("테슬라 모델 3")
                .image("abc/def")
                .build();

        CarServiceRequest request = CarServiceRequest.builder()
                .carTypeId(cartype.getId())
                .number("22나2222")
                .nickname("새로운 콩이")
                .build();

        // when, then
        assertThatThrownBy(() -> carService.updateCar(1L, user, request))
                .isInstanceOf(BaseException.class)
                .hasMessage(ErrorCode.CAR_NOT_FOUND.getMessage());
    }

    @DisplayName("차량 번호를 변경하지 않으면 차량 번호 중복 검사가 이루어지지 않는다.")
    @Test
    public void updateCarWithoutChangingNumber() {
        // given
        User user = User.builder()
                .username("test@gmail.com")
                .nickname("nickname")
                .build();

        CarType carType = CarType.builder()
                .name("테슬라 모델 3")
                .image("image/tesla3")
                .build();

        carTypeRepository.save(carType);

        Car car = Car.builder()
                .carType(carType)
                .number("11가1111")
                .nickname("콩이")
                .user(user)
                .build();

        carRepository.save(car);

        CarServiceRequest request = CarServiceRequest.builder()
                .carTypeId(carType.getId())
                .number("11가1111") // 차량 번호를 변경 x
                .nickname("새로운 콩이") // 닉네임만 변경
                .build();

        // when
        carService.updateCar(car.getId(), user, request);

        // then
        Car updatedCar = carRepository.findById(car.getId()).orElseThrow(
                () -> new BaseException(ErrorCode.CAR_NOT_FOUND)
        );

        assertThat(updatedCar.getNumber()).isEqualTo("11가1111");
        assertThat(updatedCar.getNickname()).isEqualTo("새로운 콩이");
    }


    @DisplayName("차량 번호를 변경할 때, 이미 존재하는 차량번호로 수정하면 예외가 발생한다.")
    @Test
    public void updateCarWithExistingName() {
        // given
        User user = User.builder()
                .username("test@gmail.com")
                .nickname("nickname")
                .build();

        CarType cartype = CarType.builder()
                .name("테슬라 모델 3")
                .image("abc/def")
                .build();

        Car car1 = Car.builder()
                .carType(cartype)
                .number("11가1111")
                .user(user)
                .build();

        carRepository.save(car1);

        Car car2 = Car.builder()
                .carType(cartype)
                .number("22나2222")
                .user(user)
                .build();

        carRepository.save(car2);

        CarServiceRequest request = CarServiceRequest.builder()
                .carTypeId(cartype.getId())
                .number("11가1111") // 이미 존재하는 번호로 수정 시도
                .nickname("새로운 콩이")
                .build();

        // when & then
        assertThatThrownBy(() -> carService.updateCar(car2.getId(), user, request))
                .isInstanceOf(BaseException.class)
                .hasMessage(ErrorCode.CAR_NUMBER_ALREADY_EXISTS.getMessage());
    }

    @DisplayName("존재하지 않는 차종으로 수정하면 예외가 발생한다.")
    @Test
    public void updateCarWithNonExistentCarType() {
        // given
        User user = User.builder()
                .username("test@gmail.com")
                .nickname("nickname")
                .build();

        CarType existingCarType = CarType.builder()
                .name("테슬라 모델 3")
                .image("image/tesla3")
                .build();

        carTypeRepository.save(existingCarType);

        Car car = Car.builder()
                .carType(existingCarType)
                .number("11가1111")
                .nickname("콩이")
                .user(user)
                .build();

        carRepository.save(car);

        CarServiceRequest request = CarServiceRequest.builder()
                .carTypeId(999L) // 존재하지 않을 carTypeId
                .number("11가1111")
                .nickname("순이")
                .build();

        // when & then
        assertThatThrownBy(() -> carService.updateCar(car.getId(), user, request))
                .isInstanceOf(BaseException.class)
                .hasMessage(ErrorCode.CAR_TYPE_NOT_FOUND.getMessage());
    }

    @DisplayName("차량 소유자가 아닌 사용자가 차량을 수정하려고 하면 예외가 발생한다.")
    @Test
    public void updateCarWithWrongUser() {
        // given
        User owner = User.builder()
                .username("owner@gmail.com")
                .nickname("ownerNickname")
                .build();

        User anotherUser = User.builder()
                .username("another@gmail.com")
                .nickname("anotherNickname")
                .build();

        CarType carType = CarType.builder()
                .name("테슬라 모델 3")
                .image("image/tesla3")
                .build();

        carTypeRepository.save(carType);

        Car car = Car.builder()
                .carType(carType)
                .number("11가1111")
                .nickname("콩이")
                .user(owner)
                .build();

        carRepository.save(car);

        // 수정 요청 (다른 사용자가 수정을 시도하는 상황)
        CarServiceRequest request = CarServiceRequest.builder()
                .carTypeId(carType.getId())
                .number("22나2222")
                .nickname("순이")
                .build();

        // when & then
        assertThatThrownBy(() -> carService.updateCar(car.getId(), anotherUser, request))
                .isInstanceOf(BaseException.class)
                .hasMessage(ErrorCode.CAR_NOT_BELONG_TO_USER.getMessage());

    }

    @DisplayName("차량을 삭제하면 정상적으로 삭제된다.")
    @Test
    public void deleteCar() {
        // given
        User owner = User.builder()
                .username("owner@gmail.com")
                .nickname("owner")
                .build();

        CarType carType = CarType.builder()
                .name("테슬라 모델 3")
                .image("image/tesla3")
                .build();

        carTypeRepository.save(carType);

        Car car = Car.builder()
                .carType(carType)
                .number("11가1111")
                .nickname("콩이")
                .user(owner)
                .build();

        carRepository.save(car);

        // when
        carService.deleteCar(car.getId(), owner);

        // then
        assertThatThrownBy(() -> carRepository.findById(car.getId())
                .orElseThrow(() -> new BaseException(ErrorCode.CAR_NOT_FOUND)))
                .isInstanceOf(BaseException.class)
                .hasMessage(ErrorCode.CAR_NOT_FOUND.getMessage());
    }

    @DisplayName("차량 소유자가 아닌 사용자가 차량을 삭제하려고 하면 예외가 발생한다.")
    @Test
    public void deleteCarWithWrongUser() {
        // given
        User owner = User.builder()
                .username("owner@gmail.com")
                .nickname("ownerNickname")
                .build();

        User anotherUser = User.builder()
                .username("another@gmail.com")
                .nickname("anotherNickname")
                .build();

        CarType carType = CarType.builder()
                .name("테슬라 모델 3")
                .image("image/tesla3")
                .build();

        carTypeRepository.save(carType);

        Car car = Car.builder()
                .carType(carType)
                .number("11가1111")
                .nickname("콩이")
                .user(owner)
                .build();

        carRepository.save(car);

        // when & then
        assertThatThrownBy(() -> carService.deleteCar(car.getId(), anotherUser))
                .isInstanceOf(BaseException.class)
                .hasMessage(ErrorCode.CAR_NOT_BELONG_TO_USER.getMessage());
    }

    @DisplayName("존재하지 않는 차량을 삭제하려고 하면 예외가 발생한다.")
    @Test
    public void deleteNonExistentCar() {
        // given
        User user = User.builder()
                .username("user@gmail.com")
                .nickname("user")
                .build();

        // when & then
        assertThatThrownBy(() -> carService.deleteCar(999L, user))
                .isInstanceOf(BaseException.class)
                .hasMessage(ErrorCode.CAR_NOT_FOUND.getMessage());
    }

    @DisplayName("사용자의 차량 목록을 조회하면 정상적으로 반환된다.")
    @Test
    public void getCarList() {
        // given
        User user = User.builder()
                .username("testuser@gmail.com")
                .nickname("테스트 유저")
                .build();

        CarType carType = CarType.builder()
                .name("현대 포터2 일렉트릭")
                .image("image/hyundai")
                .build();

        Car car1 = Car.builder()
                .carType(carType)
                .number("11가1111")
                .nickname("덕구")
                .user(user)
                .build();

        Car car2 = Car.builder()
                .carType(carType)
                .number("22나2222")
                .nickname("백구")
                .user(user)
                .build();

        carRepository.saveAll(List.of(car1, car2));

        LocalDateTime startOfMonth = LocalDateTime.of(2024, 9, 1, 0, 0, 0);
        LocalDateTime endOfMonth = LocalDateTime.of(2024, 9, 30, 23, 59, 59);

        // when
        List<CarListResponse> carList = carService.getCarList(user, startOfMonth, endOfMonth);

        // then
        assertThat(carList).hasSize(2)
                .extracting("number", "nickname")
                .containsExactlyInAnyOrder(
                        Tuple.tuple("11가1111", "덕구"),
                        Tuple.tuple("22나2222", "백구")
                );
    }

    @DisplayName("충전 내역을 포함한 차량 목록을 조회하면 정상적으로 반환된다.")
    @Test
    public void getCarListWithChargingLogs() {
        // given
        User user = User.builder()
                .username("testuser@gmail.com")
                .nickname("테스트 유저")
                .build();

        CarType carType1 = CarType.builder()
                .name("테슬라 모델 S")
                .image("cars/image1")
                .build();

        Car car1 = Car.builder()
                .carType(carType1)
                .number("11가1111")
                .nickname("도지")
                .user(user)
                .build();
        carRepository.save(car1);

        CarType carType2 = CarType.builder()
                .name("현대 아이오닉 5")
                .image("cars/image2")
                .build();

        Car car2 = Car.builder()
                .carType(carType2)
                .number("22나2222")
                .nickname("홍이")
                .user(user)
                .build();
        carRepository.save(car2);

        ParkingLot parkingLot = ParkingLot.builder()
                .name("수완동 지하 주차장")
                .location(Location.builder()
                        .latitude(37.5665)
                        .longitude(126.9780).build())
                .build();
        parkingLotRepository.save(parkingLot);

        Charger charger = Charger.builder()
                .parkingLot(parkingLot)
                .serialNumber("1234A")
                .battery(70)
                .status(ChargerStatus.WAITING)
                .build();
        chargerRepository.save(charger);

        ChargingLog chargingLog1Car1 = ChargingLog.builder()
                .car(car1)
                .charger(charger)
                .startTime(LocalDateTime.of(2024, 8, 23, 10, 0, 0))
                .endTime(LocalDateTime.of(2024, 8, 23, 14, 0, 0))
                .build();
        ChargingLog chargingLog2Car1 = ChargingLog.builder()
                .car(car1)
                .charger(charger)
                .startTime(LocalDateTime.of(2024, 9, 5, 10, 0, 0))
                .endTime(LocalDateTime.of(2024, 9, 5, 14, 0, 0)) // 4시간
                .build();
        ChargingLog chargingLog3Car1 = ChargingLog.builder()
                .car(car1)
                .charger(charger)
                .startTime(LocalDateTime.of(2024, 9, 6, 12, 0, 0))
                .endTime(LocalDateTime.of(2024, 9, 6, 15, 0, 0)) // 3시간
                .build();
        ChargingLog chargingLog4Car1 = ChargingLog.builder()
                .car(car1)
                .charger(charger)
                .startTime(LocalDateTime.of(2024, 9, 7, 14, 0, 0))
                .endTime(LocalDateTime.of(2024, 9, 7, 16, 0, 0)) // 2시간
                .build();
        ChargingLog chargingLog5Car1 = ChargingLog.builder()
                .car(car1)
                .charger(charger)
                .startTime(LocalDateTime.of(2024, 10, 24, 14, 0, 0))
                .endTime(LocalDateTime.of(2024, 10, 24, 16, 0, 0))
                .build();
        chargingLogRepository.saveAll(List.of(chargingLog1Car1, chargingLog2Car1, chargingLog3Car1, chargingLog4Car1, chargingLog5Car1));

        ChargingLog chargingLog1Car2 = ChargingLog.builder()
                .car(car2)
                .charger(charger)
                .startTime(LocalDateTime.of(2024, 8, 8, 10, 0, 0))
                .endTime(LocalDateTime.of(2024, 8, 8, 11, 0, 0))
                .build();
        ChargingLog chargingLog2Car2 = ChargingLog.builder()
                .car(car2)
                .charger(charger)
                .startTime(LocalDateTime.of(2024, 8, 14, 10, 0, 0))
                .endTime(LocalDateTime.of(2024, 8, 14, 11, 0, 0))
                .build();
        ChargingLog chargingLog3Car2 = ChargingLog.builder()
                .car(car2)
                .charger(charger)
                .startTime(LocalDateTime.of(2024, 9, 8, 10, 0, 0))
                .endTime(LocalDateTime.of(2024, 9, 8, 12, 0, 0)) // 2시간
                .build();
        ChargingLog chargingLog4Car2 = ChargingLog.builder()
                .car(car2)
                .charger(charger)
                .startTime(LocalDateTime.of(2024, 9, 9, 14, 0, 0))
                .endTime(LocalDateTime.of(2024, 9, 9, 16, 0, 0)) // 2시간
                .build();
        ChargingLog chargingLog5Car2 = ChargingLog.builder()
                .car(car2)
                .charger(charger)
                .startTime(LocalDateTime.of(2024, 10, 2, 14, 0, 0))
                .endTime(LocalDateTime.of(2024, 10, 2, 16, 0, 0))
                .build();
        chargingLogRepository.saveAll(List.of(chargingLog1Car2, chargingLog2Car2, chargingLog3Car2, chargingLog4Car2, chargingLog5Car2));

        LocalDateTime startOfMonth = LocalDateTime.of(2024, 9, 1, 0, 0, 0);
        LocalDateTime endOfMonth = LocalDateTime.of(2024, 9, 30, 23, 59, 59);

        // 첫 번째 차량 (9시간 충전)
        double expectedChargeAmountCar1 = 9 * CHARGE_AMOUNT_PER_HOUR;
        long expectedChargeCostCar1 = (long) (expectedChargeAmountCar1 * COST_PER_KWH);

        // 두 번째 차량 (4시간 충전)
        double expectedChargeAmountCar2 = 4 * CHARGE_AMOUNT_PER_HOUR;
        long expectedChargeCostCar2 = (long) (expectedChargeAmountCar2 * COST_PER_KWH);

        // when
        List<CarListResponse> carList = carService.getCarList(user, startOfMonth, endOfMonth);

        // then
        assertThat(carList).hasSize(2)
                .extracting("number", "chargeAmount", "chargeCost")
                .containsExactlyInAnyOrder(
                        Tuple.tuple("11가1111", expectedChargeAmountCar1, expectedChargeCostCar1),  // 첫 번째 차량 (9시간 충전)
                        Tuple.tuple("22나2222", expectedChargeAmountCar2, expectedChargeCostCar2)   // 두 번째 차량 (4시간 충전)
                );
    }

    @DisplayName("차량 충전 상태를 조회하면 정상적으로 반환된다.")
    @Test
    public void getCarChargingStatus() {
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
        CarReservationStatusResponse response = carService.getCarChargingStatus(user, car.getId());

        // then
        assertThat(response).isNotNull()
                .extracting("battery", "status", "startTime", "endTime")
                .containsExactly(0, "PENDING", LocalDateTime.of(2024, 1, 1, 0, 0), LocalDateTime.of(2024, 1, 1, 1, 0));
    }

    @DisplayName("차량 충전 상태를 조회할 때 차량이 없으면 예외가 발생한다.")
    @Test
    public void getCarChargingStatusWithoutCar() {
        // given
        User user = User.builder()
                .username("user@google.com")
                .nickname("user")
                .build();

        userRepository.save(user);

        // when, then
       assertThatThrownBy(() -> carService.getCarChargingStatus(user,1L))
               .isInstanceOf(BaseException.class)
               .hasMessage(ErrorCode.CAR_NOT_FOUND.getMessage());
    }

    @DisplayName("차량 충전 상태를 조회할 때 내 차가 아니면 예외가 발생한다.")
    @Test
    public void getCarChargingStatusWithNotMyCar() {
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

        userRepository.saveAll(List.of(user1, user2));
        carRepository.save(car);
        parkingLotRepository.save(parkingLot);
        reservationRepository.save(reservation);

        // when
        assertThatThrownBy(() -> carService.getCarChargingStatus(user1, car.getId()))
                .isInstanceOf(BaseException.class)
                .hasMessage(ErrorCode.CAR_NOT_BELONG_TO_USER.getMessage());
    }


    @DisplayName("차량 충전 상태를 조회할 예약이 없으면 null이 반환된다.")
    @Test
    public void getCarChargingStatusWithoutReservaion() {
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

        Charger charger = Charger.builder()
                .parkingLot(parkingLot)
                .serialNumber("1")
                .status(ChargerStatus.WAITING)
                .lastReservedTime(LocalDateTime.of(2024, 1, 1, 2, 0))
                .build();

        parkingLot.getChargers().add(charger);


        userRepository.saveAll(List.of(user1, user2));
        carRepository.save(car);
        parkingLotRepository.save(parkingLot);

        // when
        CarReservationStatusResponse response = carService.getCarChargingStatus(user2, car.getId());

        // then
        assertThat(response).isNull();
    }


    @DisplayName("차량을 삭제하면 관련된 ChargingLog와 Reservation도 함께 삭제된다.")
    @Test
    public void deleteCarAndRelatedEntities() {
        // given
        User user = User.builder()
                .username("testuser@gmail.com")
                .nickname("테스트 유저")
                .build();
        userRepository.save(user);

        CarType carType = CarType.builder()
                .name("테슬라 모델 3")
                .build();
        carTypeRepository.save(carType);

        Car car = Car.builder()
                .user(user)
                .carType(carType)
                .number("11가1111")
                .nickname("테슬라")
                .build();
        carRepository.save(car);

        ParkingLot parkingLot = ParkingLot.builder()
                .name("테스트 주차장")
                .location(Location.builder().latitude(37.5665).longitude(126.9780).build())
                .build();
        parkingLotRepository.save(parkingLot);

        ParkingSpot parkingSpot = ParkingSpot.builder()
                .parkingLot(parkingLot)
                .name("1")
                .location(Location.builder().latitude(37.5665).longitude(126.9780).build())
                .build();
        parkingSpotRepository.save(parkingSpot);

        Charger charger = Charger.builder()
                .parkingLot(parkingLot)
                .serialNumber("12345")
                .battery(80)
                .status(ChargerStatus.WAITING)
                .build();
        chargerRepository.save(charger);

        Reservation reservation = Reservation.create(car, parkingSpot, charger,
                LocalDateTime.of(2024, 9, 1, 10, 0), LocalDateTime.of(2024, 9, 1, 12, 0));
        reservationRepository.save(reservation);

        ChargingLog chargingLog = ChargingLog.of(reservation);
        chargingLogRepository.save(chargingLog);

        car.getReservations().add(reservation);
        car.getChargingLogs().add(chargingLog);

        // when
        carService.deleteCar(car.getId(), user);

        // then
        assertThat(carRepository.findById(car.getId())).isEmpty();
        assertThat(reservationRepository.findById(reservation.getId())).isEmpty();
        assertThat(chargingLogRepository.findById(chargingLog.getId())).isEmpty();
    }
}

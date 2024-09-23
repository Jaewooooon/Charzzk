package com.ssafy.charzzk.domain.car.service;

import com.ssafy.charzzk.IntegrationTestSupport;
import com.ssafy.charzzk.api.service.car.CarService;
import com.ssafy.charzzk.api.service.car.request.CarServiceRequest;
import com.ssafy.charzzk.api.service.car.response.CarTypeResponse;
import com.ssafy.charzzk.core.exception.BaseException;
import com.ssafy.charzzk.core.exception.ErrorCode;
import com.ssafy.charzzk.domain.car.Car;
import com.ssafy.charzzk.domain.car.CarRepository;
import com.ssafy.charzzk.domain.car.CarType;
import com.ssafy.charzzk.domain.car.CarTypeRepository;
import com.ssafy.charzzk.domain.user.User;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.test.context.ActiveProfiles;
import org.springframework.transaction.annotation.Transactional;

import java.util.List;

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
    private CarService carService;

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
}
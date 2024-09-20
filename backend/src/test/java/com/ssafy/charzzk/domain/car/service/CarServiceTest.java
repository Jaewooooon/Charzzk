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

    @DisplayName("동일한 차량번호를 등록하면 예외가 발생한다.")
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

}
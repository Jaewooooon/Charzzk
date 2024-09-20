package com.ssafy.charzzk.api.service.car;

import com.ssafy.charzzk.IntegrationTestSupport;
import com.ssafy.charzzk.api.service.car.request.CarServiceRequest;
import com.ssafy.charzzk.core.exception.BaseException;
import com.ssafy.charzzk.core.exception.ErrorCode;
import com.ssafy.charzzk.domain.car.Car;
import com.ssafy.charzzk.domain.car.CarRepository;
import com.ssafy.charzzk.domain.car.CarType;
import com.ssafy.charzzk.domain.car.CarTypeRepository;
import com.ssafy.charzzk.domain.user.User;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.test.context.ActiveProfiles;
import org.springframework.transaction.annotation.Transactional;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.junit.jupiter.api.Assertions.*;

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

    // 동일한 차량 번호
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

    // 없는 차종

}
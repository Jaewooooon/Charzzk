package com.ssafy.charzzk.domain.car.repository;

import com.ssafy.charzzk.IntegrationTestSupport;
import com.ssafy.charzzk.domain.car.Car;
import com.ssafy.charzzk.domain.car.CarRepository;
import com.ssafy.charzzk.domain.car.CarType;
import com.ssafy.charzzk.domain.user.User;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.test.context.ActiveProfiles;
import org.springframework.transaction.annotation.Transactional;

import java.util.List;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.api.Assertions.*;

@Transactional
@ActiveProfiles("test")
class CarRepositoryTest extends IntegrationTestSupport {

    @Autowired
    private CarRepository carRepository;

    @DisplayName("차량 번호가 이미 존재하는지 확인한다.")
    @Test
    public void existsByNumber() {

        User user1 = User.builder()
                .username("test1@gmail.com")
                .nickname("nickname1")
                .build();

        CarType carType = CarType.builder()
                .name("테슬라")
                .image("abc/def")
                .build();

        Car car = Car.builder()
                .user(user1)
                .carType(carType)
                .number("11가1111")
                .build();


        carRepository.save(car);

        boolean exists = carRepository.existsByNumber(car.getNumber());

        assertTrue(exists);

        exists = carRepository.existsByNumber("notExistingNumber");

        assertFalse(exists);
    }
    
    @DisplayName("특정 사용자의 차량 목록을 조회한다.")
    @Test
    public void findByUser() {
        // given
        User user1 = User.builder()
                .username("test1@gmail.com")
                .nickname("nickname1")
                .build();

        User user2 = User.builder()
                .username("test2@gmail.com")
                .nickname("nickname2")
                .build();

        CarType carType = CarType.builder()
                .name("테슬라")
                .image("abc/def")
                .build();

        Car car1 = Car.builder()
                .user(user1)
                .carType(carType)
                .number("11가1111")
                .build();

        Car car2 = Car.builder()
                .user(user1)
                .carType(carType)
                .number("22나2222")
                .build();

        Car car3 = Car.builder()
                .user(user2)
                .carType(carType)
                .number("33다3333")
                .build();

        carRepository.saveAll(List.of(car1, car2, car3));

        // when
        List<Car> cars = carRepository.findByUser(user1);

        // then
        assertThat(cars).hasSize(2)
                .extracting("number")
                .containsExactlyInAnyOrder("11가1111", "22나2222");
    }
}
package com.ssafy.charzzk.domain.car;

import com.ssafy.charzzk.IntegrationTestSupport;
import com.ssafy.charzzk.api.service.car.CarService;
import com.ssafy.charzzk.domain.user.User;
import com.ssafy.charzzk.domain.user.UserRepository;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.test.context.ActiveProfiles;
import org.springframework.transaction.annotation.Transactional;

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
}
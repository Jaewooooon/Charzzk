package com.ssafy.charzzk.domain.car.repository;

import com.ssafy.charzzk.IntegrationTestSupport;
import com.ssafy.charzzk.domain.car.CarType;
import com.ssafy.charzzk.domain.car.CarTypeRepository;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.test.context.ActiveProfiles;
import org.springframework.transaction.annotation.Transactional;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

@Transactional
@ActiveProfiles("test")
class CarTypeRepositoryTest extends IntegrationTestSupport {

    @Autowired
    private CarTypeRepository carTypeRepository;

    @BeforeEach
    public void setUp() {
        carTypeRepository.deleteAll();
    }

    @DisplayName("차종 이름에 특정 문자열이 포함된 차량 목록을 반환한다.")
    @Test
    void findByNameContaining() {

        // given
        CarType carType1 = CarType.builder()
                .name("테슬라 모델 3")
                .image("image/tesla3")
                .build();

        CarType carType2 = CarType.builder()
                .name("테슬라 모델 Y")
                .image("image/teslaY")
                .build();

        CarType carType3 = CarType.builder()
                .name("BMW X5")
                .image("image/bmwX5")
                .build();

        carTypeRepository.save(carType1);
        carTypeRepository.save(carType2);
        carTypeRepository.save(carType3);

        // when
        List<CarType> teslaTypes = carTypeRepository.findByNameContaining("테슬라");

        // then
        assertNotNull(teslaTypes);
        assertEquals(2, teslaTypes.size());
        assertTrue(teslaTypes.stream().anyMatch(carType -> carType.getName().equals("테슬라 모델 3")));
        assertTrue(teslaTypes.stream().anyMatch(carType -> carType.getName().equals("테슬라 모델 Y")));

    }
}
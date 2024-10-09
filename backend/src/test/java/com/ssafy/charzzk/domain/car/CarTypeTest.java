package com.ssafy.charzzk.domain.car;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.api.Assertions.*;

class CarTypeTest {

    @DisplayName("차종을 생성할 수 있다.")
    @Test
    public void create() {
        CarType carType = CarType.create("테스트 차종", "test.jpg");

        assertThat(carType).isNotNull()
                .extracting("name", "image")
                .containsExactly("테스트 차종", "test.jpg");

    }
}

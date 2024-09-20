package com.ssafy.charzzk.core.util;

import com.ssafy.charzzk.domain.parkinglot.Location;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

import static org.assertj.core.api.Assertions.assertThat;

class DistanceCalculatorTest {

    @DisplayName("같은 위치의 거리는 0이 나온다")
    @Test
    void calculateSameLocationDistance() {
        // given
        Location location1 = Location.builder()
                .latitude(37.123456)
                .longitude(127.123456)
                .build();

        Location location2 = Location.builder()
                .latitude(37.123456)
                .longitude(127.123456)
                .build();

        // when

        double distance = DistanceCalculator.calculateDistance(location1, location2);

        // then
        assertThat(distance).isEqualTo(0);
    }

    @DisplayName("다른 위치의 거리가 계산된다")
    @Test
    void calculateDifferentLocationDistance() {
        // given
        Location location1 = Location.builder()
                .latitude(0.0)
                .longitude(0.0)
                .build();

        Location location2 = Location.builder()
                .latitude(89.0)
                .longitude(189.0)
                .build();

        // when
        double distance = DistanceCalculator.calculateDistance(location1, location2);

        // then
        assertThat(distance).isEqualTo(1.0117369194124365E7);
    }
}
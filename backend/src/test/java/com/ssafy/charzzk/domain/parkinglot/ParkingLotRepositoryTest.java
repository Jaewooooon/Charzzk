package com.ssafy.charzzk.domain.parkinglot;

import com.ssafy.charzzk.IntegrationTestSupport;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.test.context.ActiveProfiles;
import org.springframework.transaction.annotation.Transactional;

import java.util.List;
import java.util.Optional;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.api.Assertions.assertTrue;

@Transactional
@ActiveProfiles("test")
class ParkingLotRepositoryTest extends IntegrationTestSupport {

    @Autowired
    private ParkingLotRepository parkingLotRepository;

    @DisplayName("주차장 ID로 주차장을 조회한다.")
    @Test
    void findByIdWithParkingSpots() {
        // given
        Location location1 = Location.builder()
                .latitude(0.0)
                .longitude(0.0)
                .build();

        Location location2 = Location.builder()
                .latitude(89.0)
                .longitude(179.0)
                .build();

        ParkingLot parkingLot1 = ParkingLot.builder()
                .name("주차장1")
                .location(location1)
                .build();

        ParkingLot parkingLot2 = ParkingLot.builder()
                .name("주차장2")
                .location(location2)
                .build();

        List<ParkingLot> parkingLotList = List.of(parkingLot1, parkingLot2);

        parkingLotRepository.saveAll(parkingLotList);

        // when
        Optional<ParkingLot> findParkingLot = parkingLotRepository.findByIdWithParkingSpots(parkingLot1.getId());

        // then
        assertTrue(findParkingLot.isPresent());
        assertThat(findParkingLot.get().getId()).isEqualTo(parkingLot1.getId());
    }


    @DisplayName("키워드를 포함하는 주차장 리스트를 조회한다.")
    @Test
    void findAllContaining() {
        // given
        Location location1 = Location.builder()
                .latitude(0.0)
                .longitude(0.0)
                .build();

        List<ParkingLot> parkingLotList = List.of(
                ParkingLot.builder()
                        .name("주차장1")
                        .location(location1)
                        .build(),
                ParkingLot.builder()
                        .name("주차장2")
                        .location(location1)
                        .build(),
                ParkingLot.builder()
                        .name("공영주차장")
                        .location(location1)
                        .build(),
                ParkingLot.builder()
                        .name("xxxxxxx")
                        .location(location1)
                        .build());

        parkingLotRepository.saveAll(parkingLotList);

        // when
        List<ParkingLot> findParkingLotList = parkingLotRepository.findAllContaining("주차");

        // then
        assertThat(findParkingLotList).hasSize(3)
                .extracting("name")
                .containsExactly("주차장1", "주차장2", "공영주차장");

    }
}
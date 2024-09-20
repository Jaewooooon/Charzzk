package com.ssafy.charzzk.api.service.parkinglot;

import com.ssafy.charzzk.IntegrationTestSupport;
import com.ssafy.charzzk.api.controller.parkinglot.request.ParkingLotListRequest;
import com.ssafy.charzzk.api.service.parkinglot.response.ParkingLotResponse;
import com.ssafy.charzzk.domain.parkinglot.Location;
import com.ssafy.charzzk.domain.parkinglot.ParkingLot;
import com.ssafy.charzzk.domain.parkinglot.ParkingLotRepository;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.test.context.ActiveProfiles;
import org.springframework.transaction.annotation.Transactional;

import java.util.List;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.tuple;

@Transactional
@ActiveProfiles("test")
class ParkingLotServiceTest extends IntegrationTestSupport {

    @Autowired
    private ParkingLotService parkingLotService;

    @Autowired
    private ParkingLotRepository parkingLotRepository;

    @DisplayName("주변 주차장 리스트 현재 거리와 가까운순 오름차순으로 조회한다.")
    @Test
    void getParkingLotList() {
        // given
        Location location1 = Location.builder()
                .latitude(0.0)
                .longitude(0.0)
                .build();

        Location location2 = Location.builder()
                .latitude(89.0)
                .longitude(179.0)
                .build();

        List<ParkingLot> parkingLotList = List.of(
                ParkingLot.builder()
                        .name("주차장1")
                        .location(location1)
                        .build(),
                ParkingLot.builder()
                        .name("주차장2")
                        .location(location2)
                        .build());

        ParkingLotListRequest request = ParkingLotListRequest.builder()
                .latitude(0.0)
                .longitude(0.0)
                .build();

        parkingLotRepository.saveAll(parkingLotList);

        // when
        List<ParkingLotResponse> parkingLotResponseList = parkingLotService.getParkingLotList(request);

        // then
        assertThat(parkingLotResponseList).hasSize(2)
                .extracting("id", "name", "location", "image", "parkingMapImage")
                .containsExactly(
                        tuple(parkingLotList.get(0).getId(), "주차장1", location1, null, null),
                        tuple(parkingLotList.get(1).getId(), "주차장2", location2, null, null)
                );

    }
}
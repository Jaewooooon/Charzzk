package com.ssafy.charzzk.api.service.parkinglot;

import com.ssafy.charzzk.IntegrationTestSupport;
import com.ssafy.charzzk.api.controller.parkinglot.request.ParkingLotListRequest;
import com.ssafy.charzzk.api.service.parkinglot.response.ParkingLotListResponse;
import com.ssafy.charzzk.api.service.parkinglot.response.ParkingLotResponse;
import com.ssafy.charzzk.domain.parkinglot.Location;
import com.ssafy.charzzk.domain.parkinglot.ParkingLot;
import com.ssafy.charzzk.domain.parkinglot.ParkingLotRepository;
import com.ssafy.charzzk.domain.parkinglot.ParkingSpot;
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
        List<ParkingLotListResponse> parkingLotListResponseList = parkingLotService.getParkingLotList(request, "");

        // then
        assertThat(parkingLotListResponseList).hasSize(2)
                .extracting("id", "name", "location", "image")
                .containsExactly(
                        tuple(parkingLotList.get(0).getId(), "주차장1", location1, null),
                        tuple(parkingLotList.get(1).getId(), "주차장2", location2, null)
                );
    }

    @DisplayName("검색어로 주차장을 조회하면 검색어를 포함하는 주차장 목록을 반환한다.")
    @Test
    void getParkingLotListWithKeyword() {
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
                        .build(),
                ParkingLot.builder()
                        .name("공영주차장")
                        .location(location2)
                        .build(),
                ParkingLot.builder()
                        .name("xxxxxxx")
                        .location(location2)
                        .build());

        ParkingLotListRequest request = ParkingLotListRequest.builder()
                .latitude(0.0)
                .longitude(0.0)
                .build();

        parkingLotRepository.saveAll(parkingLotList);

        // when
        List<ParkingLotListResponse> parkingLotListResponseList = parkingLotService.getParkingLotList(request, "주차장");

        // then
        assertThat(parkingLotListResponseList).hasSize(3)
                .extracting("id", "name", "location", "image")
                .containsExactly(
                        tuple(parkingLotList.get(0).getId(), "주차장1", location1, null),
                        tuple(parkingLotList.get(1).getId(), "주차장2", location2, null),
                        tuple(parkingLotList.get(2).getId(), "공영주차장", location2, null)
                );

    }


    @DisplayName("주차장 아이디로 주차장을 조회한다")
    @Test
    void getParkingLot() {

        // given
        Location location = Location.builder()
                .latitude(0.0)
                .longitude(0.0)
                .build();

        ParkingLot parkingLot = ParkingLot.builder()
                .name("주차장1")
                .parkingMapImage("주차칸 이미지")
                .location(location)
                .build();

        ParkingSpot parkingSpot1 = ParkingSpot.builder()
                .parkingLot(parkingLot)
                .location(location)
                .name("A11")
                .build();
        ParkingSpot parkingSpot2 = ParkingSpot.builder()
                .parkingLot(parkingLot)
                .location(location)
                .name("A12")
                .build();

        parkingLot.getParkingSpots().add(parkingSpot1);
        parkingLot.getParkingSpots().add(parkingSpot2);

        parkingLotRepository.save(parkingLot);

        // when
        ParkingLotResponse response = parkingLotService.getParkingLot(parkingLot.getId());

        // then
        assertThat(response).isNotNull()
                .extracting("id", "name", "parkingMapImage", "parkingSpots");
        assertThat(response.getParkingSpots()).hasSize(2)
                .extracting("id", "name")
                .containsExactlyInAnyOrder(
                        tuple(parkingSpot1.getId(), "A11"),
                        tuple(parkingSpot2.getId(), "A12")
                );
    }
}
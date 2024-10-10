package com.ssafy.charzzk.api.service.parkinglot;

import com.ssafy.charzzk.IntegrationTestSupport;
import com.ssafy.charzzk.api.service.charger.response.ChargerResponse;
import com.ssafy.charzzk.api.service.parkinglot.response.ParkingLotListResponse;
import com.ssafy.charzzk.api.service.parkinglot.response.ParkingLotResponse;
import com.ssafy.charzzk.core.exception.BaseException;
import com.ssafy.charzzk.core.exception.ErrorCode;
import com.ssafy.charzzk.domain.charger.Charger;
import com.ssafy.charzzk.domain.charger.ChargerRepository;
import com.ssafy.charzzk.domain.charger.ChargerStatus;
import com.ssafy.charzzk.domain.parkinglot.Location;
import com.ssafy.charzzk.domain.parkinglot.ParkingLot;
import com.ssafy.charzzk.domain.parkinglot.ParkingLotRepository;
import com.ssafy.charzzk.domain.parkinglot.ParkingSpot;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.test.context.ActiveProfiles;
import org.springframework.transaction.annotation.Transactional;

import java.time.LocalDateTime;
import java.util.List;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.tuple;

@Transactional
@ActiveProfiles("test")
class ParkingLotServiceTest extends IntegrationTestSupport {

    @Autowired
    private ParkingLotService parkingLotService;

    @Autowired
    private ParkingLotRepository parkingLotRepository;

    @Autowired
    private ChargerRepository chargerRepository;

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

        Double latitude = 0.0;
        Double longitude = 0.0;

        parkingLotRepository.saveAll(parkingLotList);

        String sort = "distance";
        LocalDateTime now = LocalDateTime.of(2024, 1, 1, 0, 0);

        // when
        List<ParkingLotListResponse> parkingLotListResponseList = parkingLotService.getParkingLotList(latitude, longitude, "", sort, now);

        // then
        assertThat(parkingLotListResponseList).hasSize(2)
                .extracting("id", "name", "location", "image")
                .containsExactly(
                        tuple(parkingLotList.get(0).getId(), "주차장1", location1, null),
                        tuple(parkingLotList.get(1).getId(), "주차장2", location2, null)
                );
    }

    @DisplayName("주변 주차장 리스트를 예상 대기시간이 짧은 오름차순으로 조회한다.")
    @Test
    void getParkingLotListWithWaitingTimeAscend() {
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

        Charger charger1 = Charger.builder()
                .parkingLot(parkingLot1)
                .serialNumber("11")
                .battery(80)
                .status(ChargerStatus.WAITING)
                .lastReservedTime(LocalDateTime.of(2023, 1, 1, 0, 0))
                .build();

        Charger charger2 = Charger.builder()
                .parkingLot(parkingLot2)
                .serialNumber("22")
                .battery(80)
                .status(ChargerStatus.WAITING)
                .lastReservedTime(LocalDateTime.of(2024, 1, 1, 0, 30))
                .build();

        parkingLot1.getChargers().add(charger1);
        parkingLot2.getChargers().add(charger2);

        Double latitude = 0.0;
        Double longitude = 0.0;

        parkingLotRepository.saveAll(List.of(parkingLot1, parkingLot2));

        String sort = "time";
        LocalDateTime now = LocalDateTime.of(2024, 1, 1, 0, 0);

        // when
        List<ParkingLotListResponse> parkingLotListResponseList = parkingLotService.getParkingLotList(latitude, longitude, "", sort, now);

        // then
        assertThat(parkingLotListResponseList).hasSize(2)
                .extracting("id", "name", "waitingTime")
                .containsExactly(
                        tuple(parkingLot1.getId(), "주차장1", 0),
                        tuple(parkingLot2.getId(), "주차장2", 30)
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

        Double latitude = 0.0;
        Double longitude = 0.0;

        parkingLotRepository.saveAll(parkingLotList);

        String sort = "distance";
        LocalDateTime now = LocalDateTime.of(2024, 1, 1, 0, 0);

        // when
        List<ParkingLotListResponse> parkingLotListResponseList = parkingLotService.getParkingLotList(latitude, longitude, "주차장", sort, now);

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


    @DisplayName("없는 주차장 아이디로 주차장을 조회하면 예외가 발생한다")
    @Test
    void getParkingLotWithInvalidParkingLogId() {
        // when ,then
        assertThatThrownBy(() -> parkingLotService.getParkingLot(1L))
                .isInstanceOf(BaseException.class)
                .hasMessage(ErrorCode.PARKING_LOT_NOT_FOUND.getMessage());
    }

    @DisplayName("주차장 ID로 충전 로봇 목록을 조회한다.")
    @Test
    void getChargerList() {
        // given
        ParkingLot parkingLot1 = ParkingLot.builder()
                .name("첨단 주차장")
                .location(Location.builder()
                        .latitude(37.5665)
                        .longitude(126.9780).build())
                .build();

        ParkingLot parkingLot2 = ParkingLot.builder()
                .name("상무 주차장")
                .location(Location.builder()
                        .latitude(37.5765)
                        .longitude(126.9880).build())
                .build();

        parkingLotRepository.saveAll(List.of(parkingLot1, parkingLot2));

        Charger charger1ParkingLot1 = Charger.builder()
                .parkingLot(parkingLot1)
                .serialNumber("1234A")
                .battery(80)
                .status(ChargerStatus.WAITING)
                .build();

        Charger charger2ParkingLot1 = Charger.builder()
                .parkingLot(parkingLot1)
                .serialNumber("1234B")
                .battery(90)
                .status(ChargerStatus.CHARGER_CHARGING)
                .build();

        Charger charger1ParkingLot2 = Charger.builder()
                .parkingLot(parkingLot2)
                .serialNumber("1234C")
                .battery(60)
                .status(ChargerStatus.CHARGER_CHARGING)
                .build();

        Charger charger2ParkingLot2 = Charger.builder()
                .parkingLot(parkingLot2)
                .serialNumber("1234D")
                .battery(70)
                .status(ChargerStatus.WAITING)
                .build();

        Charger charger3ParkingLot2 = Charger.builder()
                .parkingLot(parkingLot2)
                .serialNumber("1234E")
                .battery(50)
                .status(ChargerStatus.CHARGER_CHARGING)
                .build();

        // 연관관계 편의 메서드
        parkingLot1.getChargers().addAll(List.of(charger1ParkingLot1, charger2ParkingLot1));
        parkingLot2.getChargers().addAll(List.of(charger3ParkingLot2, charger2ParkingLot2, charger3ParkingLot2));

        chargerRepository.saveAll(List.of(charger1ParkingLot1, charger2ParkingLot1, charger1ParkingLot2, charger2ParkingLot2, charger3ParkingLot2));

        // when
        List<ChargerResponse> chargers = parkingLotService.getChargerList(parkingLot1.getId());

        // then
        assertThat(chargers).hasSize(2)
                .extracting("chargerId", "serialNumber", "battery", "status")
                .containsExactlyInAnyOrder(
                        tuple(charger1ParkingLot1.getId(), "1234A", 80, ChargerStatus.WAITING),
                        tuple(charger2ParkingLot1.getId(), "1234B", 90, ChargerStatus.CHARGER_CHARGING)
                );
    }

    @DisplayName("없는 주차장 아이디로 충전기 목록을 조회하면 예외가 발생한다")
    @Test
    void getChargerListWithInvalidParkingLogId() {
        // when ,then
        assertThatThrownBy(() -> parkingLotService.getChargerList(1L))
                .isInstanceOf(BaseException.class)
                .hasMessage(ErrorCode.PARKING_LOT_NOT_FOUND.getMessage());
    }
}

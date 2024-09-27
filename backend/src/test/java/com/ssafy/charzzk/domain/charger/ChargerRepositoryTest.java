package com.ssafy.charzzk.domain.charger;

import com.ssafy.charzzk.IntegrationTestSupport;
import com.ssafy.charzzk.domain.parkinglot.Location;
import com.ssafy.charzzk.domain.parkinglot.ParkingLot;
import com.ssafy.charzzk.domain.parkinglot.ParkingLotRepository;
import org.assertj.core.groups.Tuple;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.test.context.ActiveProfiles;
import org.springframework.transaction.annotation.Transactional;

import java.util.List;
import java.util.Optional;

import static org.assertj.core.api.Assertions.assertThat;

@Transactional
@ActiveProfiles("test")
class ChargerRepositoryTest extends IntegrationTestSupport {

    @Autowired
    private ChargerRepository chargerRepository;

    @Autowired
    private ParkingLotRepository parkingLotRepository;

    @Test
    @DisplayName("시리얼 넘버로 조회하면 정상적으로 충전 로봇이 조회된다.")
    void findBySerialNumber() {

        // given
        Location location = Location.builder()
                .latitude(37.5665)
                .longitude(126.9780)
                .build();

        ParkingLot parkingLot = ParkingLot.builder()
                .name("주차장 A")
                .location(location)
                .image("parking_lot_image_url")
                .parkingMapImage("parking_map_image_url")
                .build();

        parkingLotRepository.save(parkingLot);

        Charger charger = Charger.builder()
                .parkingLot(parkingLot)
                .serialNumber("12345")
                .battery(90)
                .status(ChargerStatus.WAITING)
                .build();

        chargerRepository.save(charger);

        // when
        Optional<Charger> foundCharger = chargerRepository.findBySerialNumber("12345");

        // then
        assertThat(foundCharger).isPresent();
        assertThat(foundCharger.get().getSerialNumber()).isEqualTo("12345");
        assertThat(foundCharger.get().getParkingLot().getName()).isEqualTo("주차장 A");
        assertThat(foundCharger.get().getStatus()).isEqualTo(ChargerStatus.WAITING);
        assertThat(foundCharger.get().getParkingLot().getLocation().getLatitude()).isEqualTo(37.5665);
        assertThat(foundCharger.get().getParkingLot().getLocation().getLongitude()).isEqualTo(126.9780);

    }

    @DisplayName("주차장 ID로 조회하면 해당 주차장에 속한 충전 로봇들이 정상적으로 조회된다.")
    @Test
    void findByParkingLotId() {
        // given
        ParkingLot parkingLot1 = ParkingLot.builder()
                .name("첨단 최첨단 주차장")
                .location(Location.builder()
                        .latitude(37.5665)
                        .longitude(126.9780).build())
                .image("parking_lot_image_url1")
                .parkingMapImage("parking_map_image_url1")
                .build();
        parkingLotRepository.save(parkingLot1);

        ParkingLot parkingLot2 = ParkingLot.builder()
                .name("장덕 워리어 주차장")
                .location(Location.builder()
                        .latitude(37.5465)
                        .longitude(126.8780).build())
                .image("parking_lot_image_url2")
                .parkingMapImage("parking_map_image_url2")
                .build();
        parkingLotRepository.save(parkingLot2);

        Charger charger1 = Charger.builder()
                .parkingLot(parkingLot1)
                .serialNumber("1234A")
                .battery(80)
                .status(ChargerStatus.WAITING)
                .build();

        Charger charger2 = Charger.builder()
                .parkingLot(parkingLot1)
                .serialNumber("1234B")
                .battery(70)
                .status(ChargerStatus.STOP)
                .build();

        Charger charger3 = Charger.builder()
                .parkingLot(parkingLot2)
                .serialNumber("1234C")
                .battery(50)
                .status(ChargerStatus.CHARGER_CHARGING)
                .build();

        chargerRepository.saveAll(List.of(charger1, charger2, charger3));

        // when
        List<Charger> chargers = chargerRepository.findByParkingLotId(parkingLot1.getId());

        // then
        assertThat(chargers).hasSize(2)
                .extracting("serialNumber", "battery", "status")
                .containsExactlyInAnyOrder(
                        Tuple.tuple("1234A", 80, ChargerStatus.WAITING),
                        Tuple.tuple("1234B", 70, ChargerStatus.STOP)
                );
    }

}
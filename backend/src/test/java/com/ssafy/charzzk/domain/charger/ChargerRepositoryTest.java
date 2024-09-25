package com.ssafy.charzzk.domain.charger;

import com.ssafy.charzzk.IntegrationTestSupport;
import com.ssafy.charzzk.domain.parkinglot.Location;
import com.ssafy.charzzk.domain.parkinglot.ParkingLot;
import com.ssafy.charzzk.domain.parkinglot.ParkingLotRepository;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.test.context.ActiveProfiles;
import org.springframework.transaction.annotation.Transactional;

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

}
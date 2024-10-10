package com.ssafy.charzzk.api.service.parkinglot;

import com.ssafy.charzzk.api.service.charger.response.ChargerResponse;
import com.ssafy.charzzk.api.service.parkinglot.response.ParkingLotListResponse;
import com.ssafy.charzzk.api.service.parkinglot.response.ParkingLotResponse;
import com.ssafy.charzzk.core.exception.BaseException;
import com.ssafy.charzzk.core.exception.ErrorCode;
import com.ssafy.charzzk.core.util.DistanceCalculator;
import com.ssafy.charzzk.domain.charger.Charger;
import com.ssafy.charzzk.domain.parkinglot.Location;
import com.ssafy.charzzk.domain.parkinglot.ParkingLot;
import com.ssafy.charzzk.domain.parkinglot.ParkingLotRepository;
import com.ssafy.charzzk.domain.reservation.Reservation;
import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

import java.time.LocalDateTime;
import java.time.temporal.ChronoUnit;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Collectors;

@Transactional(readOnly = true)
@RequiredArgsConstructor
@Service
public class ParkingLotService {

    private final ParkingLotRepository parkingLotRepository;


    public List<ParkingLotListResponse> getParkingLotList(Double latitude, Double longitude, String keyword, String sort, LocalDateTime now) {
        List<ParkingLot> parkingLotList = parkingLotRepository.findAllContaining(keyword);

        Location currentLocation = Location.builder()
                .latitude(latitude)
                .longitude(longitude)
                .build();

        return parkingLotList.stream()
                .map(parkingLot -> {
                    LocalDateTime earliestEndTime = parkingLot.getChargers().stream()
                            .map(Charger::getLastReservedTime)
                            .min(LocalDateTime::compareTo)
                            .orElse(LocalDateTime.MAX);

                    int waitingTime;
                    if (now.isAfter(earliestEndTime)) {
                        waitingTime = 0;
                    } else {
                        waitingTime = (int) ChronoUnit.MINUTES.between(now, earliestEndTime);
                    }

                    Location parkingLotLocation = parkingLot.getLocation();
                    double distance = DistanceCalculator.calculateDistance(currentLocation, parkingLotLocation);

                    return ParkingLotListResponse.of(parkingLot, distance, waitingTime);
                })
                .sorted((p1, p2) -> {
                    if ("time".equals(sort)) {
                        return Integer.compare(p1.getWaitingTime(), p2.getWaitingTime());
                    } else {
                        return Double.compare(p1.getDistance(), p2.getDistance());
                    }
                })
                .toList();
    }

    public ParkingLotResponse getParkingLot(Long parkingLotId) {
        ParkingLot findParkingLot = parkingLotRepository.findByIdWithParkingSpots(parkingLotId)
                .orElseThrow(() -> new BaseException(ErrorCode.PARKING_LOT_NOT_FOUND));

        return ParkingLotResponse.of(findParkingLot);
    }

    public List<ChargerResponse> getChargerList(Long parkingLotId) {
        ParkingLot parkingLot = parkingLotRepository.findByIdWithChargers(parkingLotId)
                .orElseThrow(() -> new BaseException(ErrorCode.PARKING_LOT_NOT_FOUND));

        return parkingLot.getChargers().stream()
                .map(charger -> {
                    List<Reservation> reservations = charger.getReservations().stream()
                            .filter(Reservation::inUsing)
                            .sorted(Comparator.comparing(Reservation::getStartTime))
                            .toList();

                    return ChargerResponse.of(charger, reservations);
                })
                .collect(Collectors.toList());
    }

}

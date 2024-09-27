package com.ssafy.charzzk.api.service.parkinglot;

import com.ssafy.charzzk.api.controller.parkinglot.request.ParkingLotListRequest;
import com.ssafy.charzzk.api.service.charger.response.ChargerResponse;
import com.ssafy.charzzk.api.service.parkinglot.response.ParkingLotListResponse;
import com.ssafy.charzzk.api.service.parkinglot.response.ParkingLotResponse;
import com.ssafy.charzzk.core.exception.BaseException;
import com.ssafy.charzzk.core.exception.ErrorCode;
import com.ssafy.charzzk.core.util.DistanceCalculator;
import com.ssafy.charzzk.domain.charger.Charger;
import com.ssafy.charzzk.domain.charger.ChargerRepository;
import com.ssafy.charzzk.domain.parkinglot.Location;
import com.ssafy.charzzk.domain.parkinglot.ParkingLot;
import com.ssafy.charzzk.domain.parkinglot.ParkingLotRepository;
import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

import java.util.Comparator;
import java.util.List;
import java.util.stream.Collectors;

@Transactional(readOnly = true)
@RequiredArgsConstructor
@Service
public class ParkingLotService {

    private final ParkingLotRepository parkingLotRepository;

    private final ChargerRepository chargerRepository;

    //    public List<ParkingLotListResponse> getParkingLotList(ParkingLotListRequest request, String keyword) {
//        List<ParkingLot> ParkingLotList = parkingLotRepository.findAllContaining(keyword);
//
//        Location currentLocation = Location.builder()
//                .latitude(request.getLatitude())
//                .longitude(request.getLongitude())
//                .build();
//
//
//        return ParkingLotList.stream()
//                .map(parkingLot -> {
//                    Location parkingLotLocation = parkingLot.getLocation();
//                    double distance = DistanceCalculator.calculateDistance(currentLocation, parkingLotLocation);
//
//                    return ParkingLotListResponse.of(parkingLot, distance);
//                })
//                .sorted(Comparator.comparingDouble(ParkingLotListResponse::getDistance))
//                .toList();
//    }

    public List<ParkingLotListResponse> getParkingLotList(Double latitude, Double longitude, String keyword) {
        List<ParkingLot> ParkingLotList = parkingLotRepository.findAllContaining(keyword);

        Location currentLocation = Location.builder()
                .latitude(latitude)
                .longitude(longitude)
                .build();

        return ParkingLotList.stream()
                .map(parkingLot -> {
                    Location parkingLotLocation = parkingLot.getLocation();
                    double distance = DistanceCalculator.calculateDistance(currentLocation, parkingLotLocation);

                    return ParkingLotListResponse.of(parkingLot, distance);
                })
                .sorted(Comparator.comparingDouble(ParkingLotListResponse::getDistance))
                .toList();
    }

    public ParkingLotResponse getParkingLot(Long parkingLotId) {
        ParkingLot findParkingLot = parkingLotRepository.findByIdWithParkingSpots(parkingLotId)
                .orElseThrow(() -> new BaseException(ErrorCode.PARKING_LOT_NOT_FOUND));

        return ParkingLotResponse.of(findParkingLot);
    }

    public List<ChargerResponse> getChargerList(Long parkingLotId) {
        List<Charger> chargerList = chargerRepository.findByParkingLotId(parkingLotId);
        return chargerList.stream()
                .map(ChargerResponse::from)
                .collect(Collectors.toList());
    }
}

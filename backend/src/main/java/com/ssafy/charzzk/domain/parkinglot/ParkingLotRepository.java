package com.ssafy.charzzk.domain.parkinglot;

import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Query;

import java.util.List;
import java.util.Optional;

public interface ParkingLotRepository extends JpaRepository<ParkingLot, Long> {

    @Query("SELECT p FROM ParkingLot p LEFT JOIN FETCH p.parkingSpots WHERE p.id = :parkingLotId")
    Optional<ParkingLot> findByIdWithParkingSpots(Long parkingLotId);

    @Query("SELECT p FROM ParkingLot p WHERE p.name LIKE %:keyword%")
    List<ParkingLot> findAllContaining(String keyword);

    @Query("SELECT p FROM ParkingLot p LEFT JOIN FETCH p.chargers WHERE p.id = :parkingLotId")
    Optional<ParkingLot> findByIdWithChargers(Long parkingLotId);
}

package com.ssafy.charzzk.domain.charger;

import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Query;

import java.util.Optional;

public interface ChargerRepository extends JpaRepository<Charger, Long> {

    @Query("SELECT c FROM Charger c JOIN FETCH c.parkingLot WHERE c.serialNumber = :serialNumber")
    Optional<Charger> findBySerialNumber(String serialNumber);
}

package com.ssafy.charzzk.domain.charger;

import org.springframework.data.jpa.repository.JpaRepository;

import java.util.Optional;

public interface ChargerRepository extends JpaRepository<Charger, Long> {
    Optional<Charger> findBySerialNumber(String serialNumber);
}

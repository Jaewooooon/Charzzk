package com.ssafy.charzzk.domain.reservation;

import org.springframework.data.jpa.repository.JpaRepository;

import java.util.Optional;

public interface ReservationRepository extends JpaRepository<Reservation, Long> {
    Optional<Reservation> findFirstByChargerIdOrderByEndTimeDesc(Long id);
}

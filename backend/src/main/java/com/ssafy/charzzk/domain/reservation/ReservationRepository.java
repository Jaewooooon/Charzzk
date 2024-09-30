package com.ssafy.charzzk.domain.reservation;

import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Query;

import java.util.Optional;

public interface ReservationRepository extends JpaRepository<Reservation, Long> {
    Optional<Reservation> findFirstByChargerIdOrderByEndTimeDesc(Long id);

    @Query("SELECT r FROM Reservation r JOIN FETCH r.car c JOIN FETCH c.carType WHERE r.id = :reservationId")
    Optional<Reservation> findByIdWithCar(Long reservationId);
}

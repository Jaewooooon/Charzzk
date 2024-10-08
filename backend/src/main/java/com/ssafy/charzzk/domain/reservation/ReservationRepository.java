package com.ssafy.charzzk.domain.reservation;

import com.ssafy.charzzk.domain.car.Car;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Query;
import org.springframework.data.repository.query.Param;

import java.util.List;
import java.util.Optional;

public interface ReservationRepository extends JpaRepository<Reservation, Long> {
    Optional<Reservation> findFirstByChargerIdOrderByEndTimeDesc(Long id);

    @Query("SELECT r FROM Reservation r JOIN FETCH r.car c JOIN FETCH c.carType WHERE r.id = :reservationId")
    Optional<Reservation> findByIdWithCar(Long reservationId);

    @Query("SELECT r FROM Reservation r JOIN FETCH r.car c JOIN FETCH c.user JOIN FETCH r.charger WHERE r.id = :reservationId")
    Optional<Reservation> findByIdWithCarAndCharger(Long reservationId);

    @Query(value = "SELECT * FROM reservation r WHERE r.car_id = :carId ORDER BY r.updated_at DESC LIMIT 1", nativeQuery = true)
    Optional<Reservation> findLatestReservationByCar(@Param("carId") Long carId);

}

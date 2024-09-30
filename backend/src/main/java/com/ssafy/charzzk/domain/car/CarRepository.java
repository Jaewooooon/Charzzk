package com.ssafy.charzzk.domain.car;

import jakarta.validation.constraints.NotBlank;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Query;

import java.util.Optional;

public interface CarRepository extends JpaRepository<Car, Long> {

    @Query("SELECT c FROM Car c JOIN FETCH c.user JOIN FETCH c.carType WHERE c.id = :carId")
    Optional<Car> findById(Long carId);

    boolean existsByNumber(String number);
}

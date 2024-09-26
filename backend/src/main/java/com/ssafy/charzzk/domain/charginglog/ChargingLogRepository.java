package com.ssafy.charzzk.domain.charginglog;

import com.ssafy.charzzk.domain.car.Car;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Query;

import java.time.LocalDateTime;
import java.util.List;

public interface ChargingLogRepository extends JpaRepository<ChargingLog, Long> {

    @Query("SELECT cl FROM ChargingLog cl WHERE cl.car = :car AND cl.startTime >= :startTime AND cl.endTime <= :endTime")
    List<ChargingLog> findByCarAndTimePeriod(Car car, LocalDateTime startTime, LocalDateTime endTime);
}

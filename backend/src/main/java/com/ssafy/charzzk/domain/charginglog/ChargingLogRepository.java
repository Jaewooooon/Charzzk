package com.ssafy.charzzk.domain.charginglog;

import com.ssafy.charzzk.domain.car.Car;
import com.ssafy.charzzk.domain.user.User;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Query;

import java.time.LocalDateTime;
import java.util.List;

public interface ChargingLogRepository extends JpaRepository<ChargingLog, Long> {

    @Query("SELECT cl FROM ChargingLog cl WHERE cl.car = :car AND cl.endTime BETWEEN :startOfMonth AND :endOfMonth")
    List<ChargingLog> findByCarAndEndTime(Car car, LocalDateTime startOfMonth, LocalDateTime endOfMonth);

    @Query("SELECT cl FROM ChargingLog cl JOIN FETCH cl.car c JOIN FETCH c.carType WHERE c.user = :user")
    List<ChargingLog> findByUser(User user);
}

package com.ssafy.charzzk.domain.report;

import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Query;

import java.util.Optional;

public interface ReportRepository extends JpaRepository<Report, Long> {

    @Query("SELECT r FROM Report r JOIN FETCH r.user JOIN FETCH r.parkingLot WHERE r.id = :reportId")
    Optional<Report> findById(Long reportId);
}

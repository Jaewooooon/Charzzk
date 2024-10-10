package com.ssafy.charzzk.domain.car;

import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Query;
import org.springframework.data.repository.query.Param;

import java.util.List;

public interface CarTypeRepository extends JpaRepository<CarType, Long> {

    @Query("SELECT c FROM CarType c WHERE c.name LIKE %:keyword%")
    List<CarType> findByNameContaining(@Param("keyword") String keyword);
}

package com.ssafy.charzzk.domain.reservation;

import com.ssafy.charzzk.domain.BaseEntity;
import com.ssafy.charzzk.domain.car.Car;
import com.ssafy.charzzk.domain.charger.Charger;
import jakarta.persistence.*;
import lombok.*;

import java.time.LocalTime;

@Entity
@Getter
@NoArgsConstructor(access = AccessLevel.PROTECTED)
public class Reservation extends BaseEntity {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    @Column(name = "reservation_id")
    private Long id;

    @ManyToOne(fetch = FetchType.LAZY)
    @JoinColumn(name = "car_id", nullable = false)
    private Car car;

    @ManyToOne(fetch = FetchType.LAZY)
    @JoinColumn(name = "charger_id", nullable = false)
    private Charger charger;

    @Column(name = "start_time", nullable = false)
    private LocalTime startTime;

    @Column(name = "end_time", nullable = false)
    private LocalTime endTime;

    @Builder
    private Reservation(Car car, Charger charger, LocalTime startTime, LocalTime endTime) {
        this.car = car;
        this.charger = charger;
        this.startTime = startTime;
        this.endTime = endTime;
    }
}

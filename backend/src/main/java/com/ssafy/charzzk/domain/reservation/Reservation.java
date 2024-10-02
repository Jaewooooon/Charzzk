package com.ssafy.charzzk.domain.reservation;

import com.ssafy.charzzk.domain.BaseEntity;
import com.ssafy.charzzk.domain.car.Car;
import com.ssafy.charzzk.domain.charger.Charger;
import jakarta.persistence.*;
import lombok.*;

import java.time.LocalDateTime;

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
    private LocalDateTime startTime;

    @Column(name = "end_time", nullable = false)
    private LocalDateTime endTime;

    @Enumerated(EnumType.STRING)
    private ReservationStatus status;


    @Builder
    private Reservation(Long id, Car car, Charger charger, LocalDateTime startTime, LocalDateTime endTime, ReservationStatus status) {
        this.id = id;
        this.car = car;
        this.charger = charger;
        this.startTime = startTime;
        this.endTime = endTime;
        this.status = status;
    }

    public static Reservation create(Car car, Charger charger, LocalDateTime startTime, LocalDateTime endTime) {

        return Reservation.builder()
                .car(car)
                .charger(charger)
                .startTime(startTime)
                .endTime(endTime)
                .status(ReservationStatus.PENDING)
                .build();
    }

    public void confirm() {
        this.status = ReservationStatus.WAITING;
    }
}

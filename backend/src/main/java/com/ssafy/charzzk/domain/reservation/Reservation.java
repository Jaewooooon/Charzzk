package com.ssafy.charzzk.domain.reservation;

import com.ssafy.charzzk.domain.BaseEntity;
import com.ssafy.charzzk.domain.car.Car;
import com.ssafy.charzzk.domain.charger.Charger;
import com.ssafy.charzzk.domain.parkinglot.ParkingSpot;
import jakarta.persistence.*;
import lombok.*;

import java.time.LocalDateTime;
import java.util.List;
import java.util.Objects;

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

    @ManyToOne(fetch = FetchType.LAZY, cascade = CascadeType.PERSIST)
    @JoinColumn(name = "parkingSpot_id", nullable = false)
    private ParkingSpot parkingSpot;

    @Column(name = "start_time", nullable = false)
    private LocalDateTime startTime;

    @Column(name = "end_time", nullable = false)
    private LocalDateTime endTime;

    @Enumerated(EnumType.STRING)
    private ReservationStatus status;

    @Override
    public String toString() {
        return "Reservation{" +
                "id=" + id +
                ", car=" + car.getNumber() +
                ", charger=" + charger.getId() +
                ", parkingSpot=" + parkingSpot.getId() +
                ", status=" + status +
                '}';
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Reservation that = (Reservation) o;
        return Objects.equals(id, that.id);
    }

    @Override
    public int hashCode() {
        return Objects.hashCode(id);
    }

    @Builder
    private Reservation(Long id, Car car, Charger charger, ParkingSpot parkingSpot, LocalDateTime startTime, LocalDateTime endTime, ReservationStatus status) {
        this.id = id;
        this.car = car;
        this.charger = charger;
        this.parkingSpot = parkingSpot;
        this.startTime = startTime;
        this.endTime = endTime;
        this.status = status;
    }

    public static Reservation create(Car car, ParkingSpot parkingSpot, Charger charger, LocalDateTime startTime, LocalDateTime endTime) {

        return Reservation.builder()
                .car(car)
                .parkingSpot(parkingSpot)
                .charger(charger)
                .startTime(startTime)
                .endTime(endTime)
                .status(ReservationStatus.PENDING)
                .build();
    }

    public void confirm() {
        this.status = ReservationStatus.WAITING;
    }

    public void start() {
        this.status = ReservationStatus.CHARGING;
    }

    public void cancel() {
        this.status = ReservationStatus.CANCELED;
    }

    public boolean inUsing() {
        return List.of(ReservationStatus.WAITING, ReservationStatus.CHARGING).contains(this.status);
    }

    public void chargeComplete() {
        this.status = ReservationStatus.DONE;
        charger.chargeComplete();
        car.chargeComplete();
    }
}

package com.ssafy.charzzk.domain.charger;

import com.ssafy.charzzk.domain.BaseEntity;
import com.ssafy.charzzk.domain.parkinglot.ParkingLot;
import com.ssafy.charzzk.domain.reservation.Reservation;
import jakarta.persistence.*;
import lombok.AccessLevel;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.List;

@Entity
@Table(name = "charger", uniqueConstraints = {
        @UniqueConstraint(columnNames = "serial_number")
})
@Getter
@NoArgsConstructor(access = AccessLevel.PROTECTED)
public class Charger extends BaseEntity {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    @Column(name = "charger_id")
    private Long id;

    @ManyToOne(fetch = FetchType.LAZY)
    @JoinColumn(name = "parking_lot_id", nullable = false)
    private ParkingLot parkingLot;

    private String serialNumber;

    private Integer battery;

    @Enumerated(EnumType.STRING)
    private ChargerStatus status;

    private LocalDateTime lastReservedTime = LocalDateTime.MIN;

    @OneToMany(fetch = FetchType.LAZY, mappedBy = "charger")
    private List<Reservation> reservations = new ArrayList<>();

    @Builder
    private Charger(Long id, ParkingLot parkingLot, String serialNumber, Integer battery, ChargerStatus status, LocalDateTime lastReservedTime) {
        this.id = id;
        this.parkingLot = parkingLot;
        this.serialNumber = serialNumber;
        this.battery = battery;
        this.status = status;
        this.lastReservedTime = lastReservedTime;
    }

    public void updateLastReservedTime(LocalDateTime startTime) {
        this.lastReservedTime = startTime;
    }

    public void setParkingLot(ParkingLot parkingLot) {
        this.parkingLot = parkingLot;
        parkingLot.getChargers().add(this);
    }

    public void startCharge() {
        this.status = ChargerStatus.CAR_CHARGING;
    }

    public void stopCharge() {
        this.status = ChargerStatus.WAITING;
    }

    public void chargeComplete() {
        this.status = ChargerStatus.WAITING;
    }
}

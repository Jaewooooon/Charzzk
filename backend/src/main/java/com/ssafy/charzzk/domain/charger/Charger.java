package com.ssafy.charzzk.domain.charger;

import com.ssafy.charzzk.domain.BaseEntity;
import com.ssafy.charzzk.domain.parkinglot.ParkingLot;
import jakarta.persistence.Column;
import jakarta.persistence.Entity;
import jakarta.persistence.EnumType;
import jakarta.persistence.Enumerated;
import jakarta.persistence.FetchType;
import jakarta.persistence.GeneratedValue;
import jakarta.persistence.GenerationType;
import jakarta.persistence.Id;
import jakarta.persistence.JoinColumn;
import jakarta.persistence.ManyToOne;
import jakarta.persistence.Table;
import jakarta.persistence.UniqueConstraint;
import lombok.AccessLevel;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

import java.time.LocalDateTime;

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

    @Builder
    private Charger(ParkingLot parkingLot, String serialNumber, Integer battery, ChargerStatus status, LocalDateTime lastReservedTime) {
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
}

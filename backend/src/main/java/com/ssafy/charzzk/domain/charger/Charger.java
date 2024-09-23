package com.ssafy.charzzk.domain.charger;

import com.ssafy.charzzk.domain.BaseEntity;
import com.ssafy.charzzk.domain.parkinglot.ParkingLot;
import jakarta.persistence.*;
import lombok.AccessLevel;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

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

    @Column(name = "serial_number")
    private String serialNumber;

    @Column(name = "battery")
    private Integer battery;

    @Enumerated(EnumType.STRING)
    private ChargerStatus status;

    @Builder
    private Charger(ParkingLot parkingLot, String serialNumber, Integer battery, ChargerStatus status) {
        this.parkingLot = parkingLot;
        this.serialNumber = serialNumber;
        this.battery = battery;
        this.status = status;
    }

}

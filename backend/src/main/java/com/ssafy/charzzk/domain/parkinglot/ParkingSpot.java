package com.ssafy.charzzk.domain.parkinglot;

import com.ssafy.charzzk.domain.BaseEntity;
import jakarta.persistence.*;
import lombok.AccessLevel;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Entity
@Getter
@NoArgsConstructor(access = AccessLevel.PROTECTED)
public class ParkingSpot {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    @Column(name = "parking_spot_id")
    private Long id;

    @ManyToOne(fetch = FetchType.LAZY)
    @JoinColumn(name = "parking_lot_id", nullable = false)
    private ParkingLot parkingLot;

    @Column(nullable = false)
    private String name;

    @Embedded
    private Location location;

    @Builder
    private ParkingSpot(ParkingLot parkingLot, String name, Location location) {
        this.parkingLot = parkingLot;
        this.name = name;
        this.location = location;
    }
}

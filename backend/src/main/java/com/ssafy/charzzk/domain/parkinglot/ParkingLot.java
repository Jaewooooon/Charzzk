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
public class ParkingLot extends BaseEntity {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    @Column(name = "parking_lot_id")
    private Long id;

    @Column(name = "name", nullable = false)
    private String name;

    @Embedded
    private Location location;

    private String image;

    private String parkingMapImage;

    @Builder
    private ParkingLot(String name, Location location, String image, String parkingMapImage) {
        this.name = name;
        this.location = location;
        this.image = image;
        this.parkingMapImage = parkingMapImage;
    }


}

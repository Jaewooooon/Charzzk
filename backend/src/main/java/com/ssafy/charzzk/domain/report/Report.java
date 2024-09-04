package com.ssafy.charzzk.domain.report;

import com.ssafy.charzzk.domain.BaseEntity;
import com.ssafy.charzzk.domain.parkinglot.ParkingLot;
import com.ssafy.charzzk.domain.user.User;
import jakarta.persistence.*;
import lombok.AccessLevel;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Entity
@Getter
@NoArgsConstructor(access = AccessLevel.PROTECTED)
public class Report extends BaseEntity {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    @Column(name = "report_id")
    private Long id;

    @ManyToOne(fetch = FetchType.LAZY)
    @JoinColumn(name = "user_id", nullable = false)
    private User user;

    @ManyToOne(fetch = FetchType.LAZY)
    @JoinColumn(name = "parking_lot_id", nullable = false)
    private ParkingLot parkingLot;

    @Builder
    private Report(User user, ParkingLot parkingLot) {
        this.user = user;
        this.parkingLot = parkingLot;
    }

}

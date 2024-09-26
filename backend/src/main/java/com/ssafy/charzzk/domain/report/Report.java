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
    @JoinColumn(name = "parking_lot_id")
    private ParkingLot parkingLot;

    @Enumerated(EnumType.STRING)
    @Column(name = "type", nullable = false)
    private ReportType type;

    @Column(nullable = false)
    private String content;

    @Column(nullable = true)
    private String image;

    @Column(nullable = false)
    private boolean isRead;


    @Builder
    private Report(User user, ParkingLot parkingLot, ReportType type, String content, String image, boolean isRead) {
        this.user = user;
        this.parkingLot = parkingLot;
        this.type = type;
        this.content = content;
        this.image = image;
        this.isRead = isRead;
    }

    public static Report create(User user, ParkingLot parkingLot, ReportType type, String content, String image) {
        return Report.builder()
                .user(user)
                .parkingLot(parkingLot)
                .type(type)
                .content(content)
                .image(image)
                .isRead(false)
                .build();
    }

    public void readReport() {
        this.isRead = true;
    }

}

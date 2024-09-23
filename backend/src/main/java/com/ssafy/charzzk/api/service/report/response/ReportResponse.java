package com.ssafy.charzzk.api.service.report.response;

import com.ssafy.charzzk.api.service.user.response.UserResponse;
import com.ssafy.charzzk.domain.parkinglot.ParkingLot;
import com.ssafy.charzzk.domain.report.Report;
import com.ssafy.charzzk.domain.report.ReportType;
import com.ssafy.charzzk.domain.user.User;
import lombok.Builder;
import lombok.Getter;

import java.time.LocalDateTime;

@Getter
public class ReportResponse {

    private Long id;
    private UserResponse user;
    private ReportType reportType;
    private ParkingLot parkingLot;
    private String content;
    private String image;

    private LocalDateTime createdAt;

    @Builder
    private ReportResponse(Long id, UserResponse user, ReportType reportType, ParkingLot parkingLot, String content, String image, LocalDateTime createdAt) {
        this.id = id;
        this.user = user;
        this.reportType = reportType;
        this.parkingLot = parkingLot;
        this.content = content;
        this.image = image;
        this.createdAt = createdAt;
    }

    public static ReportResponse from(Report report) {
        return ReportResponse.builder()
                .id(report.getId())
                .user(UserResponse.of(report.getUser()))
                .reportType(report.getType())
                .parkingLot(report.getParkingLot())
                .content(report.getContent())
                .image(report.getImage())
                .createdAt(report.getCreatedAt())
                .build();
    }
}

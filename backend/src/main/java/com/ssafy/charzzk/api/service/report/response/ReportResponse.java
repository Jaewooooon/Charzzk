package com.ssafy.charzzk.api.service.report.response;

import com.fasterxml.jackson.annotation.JsonFormat;
import com.ssafy.charzzk.api.service.parkinglot.response.ParkingLotReportResponse;
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
    private ParkingLotReportResponse parkingLot;
    private String content;
    private String image;
    private boolean isRead;

    @JsonFormat(shape = JsonFormat.Shape.STRING, pattern = "yyyy-MM-dd'T'HH:mm:ss")
    private LocalDateTime createdAt;

    @Builder
    private ReportResponse(Long id, UserResponse user, ReportType reportType, ParkingLotReportResponse parkingLot, String content, String image, boolean isRead, LocalDateTime createdAt) {
        this.id = id;
        this.user = user;
        this.reportType = reportType;
        this.parkingLot = parkingLot;
        this.content = content;
        this.image = image;
        this.isRead = isRead;
        this.createdAt = createdAt;
    }

    public static ReportResponse from(Report report) {
        return ReportResponse.builder()
                .id(report.getId())
                .user(UserResponse.of(report.getUser()))
                .reportType(report.getType())
                .parkingLot(ParkingLotReportResponse.of(report.getParkingLot()))
                .content(report.getContent())
                .image(report.getImage())
                .isRead(report.isRead())
                .createdAt(report.getCreatedAt())
                .build();
    }
}

package com.ssafy.charzzk.api.service.report.response;

import com.fasterxml.jackson.annotation.JsonFormat;
import com.ssafy.charzzk.api.service.parkinglot.response.ParkingLotReportResponse;
import com.ssafy.charzzk.api.service.user.response.UserResponse;
import com.ssafy.charzzk.domain.report.Report;
import com.ssafy.charzzk.domain.report.ReportType;
import lombok.Builder;
import lombok.Getter;

import java.time.LocalDateTime;

@Getter
public class ReportListResponse {

    private Long id;
    private UserResponse user;
    private ReportType reportType;
    private ParkingLotReportResponse parkingLot;
    private boolean isRead;
    @JsonFormat(shape = JsonFormat.Shape.STRING, pattern = "yyyy-MM-dd'T'HH:mm:ss")
    private LocalDateTime createdAt;

    @Builder
    private ReportListResponse(Long id, UserResponse user, ReportType reportType, ParkingLotReportResponse parkingLot, boolean isRead, LocalDateTime createdAt) {
        this.id = id;
        this.user = user;
        this.reportType = reportType;
        this.parkingLot = parkingLot;
        this.isRead = isRead;
        this.createdAt = createdAt;
    }

    public static ReportListResponse from(Report report) {
        return ReportListResponse.builder()
                .id(report.getId())
                .user(UserResponse.of(report.getUser()))
                .reportType(report.getType())
                .parkingLot(ParkingLotReportResponse.of(report.getParkingLot()))
                .isRead(report.isRead())
                .createdAt(report.getCreatedAt())
                .build();
    }


}

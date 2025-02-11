package com.ssafy.charzzk.api.controller.report;

import com.ssafy.charzzk.api.ApiResponse;
import com.ssafy.charzzk.api.controller.report.request.ReportRequest;
import com.ssafy.charzzk.api.service.report.ReportService;
import com.ssafy.charzzk.api.service.report.response.ReportListResponse;
import com.ssafy.charzzk.api.service.report.response.ReportResponse;
import com.ssafy.charzzk.core.annotation.CurrentUser;
import com.ssafy.charzzk.domain.user.User;
import jakarta.validation.Valid;
import lombok.RequiredArgsConstructor;
import org.springframework.web.bind.annotation.*;
import org.springframework.web.multipart.MultipartFile;

import java.util.List;

@RequiredArgsConstructor
@RestController
public class ReportController {

    private final ReportService reportService;

    @PostMapping("/api/v1/reports")
    public ApiResponse<ReportResponse> createReport(
            @CurrentUser User user,
            @RequestPart("report") @Valid ReportRequest request,
            @RequestPart(value = "image", required = false) MultipartFile image
    ) {
        Long reportId = reportService.createReport(user, request.toServiceRequest(), image);
        return ApiResponse.ok(reportService.getReport(reportId));
    }

    @PatchMapping("/api/v1/reports/{reportId}")
    public ApiResponse<ReportResponse> readReport(
            @CurrentUser User user,
            @PathVariable Long reportId
    ) {
        reportService.readReport(user, reportId);
        return ApiResponse.ok(reportService.getReport(reportId));
    }

    @GetMapping("/api/v1/reports/{reportId}")
    public ApiResponse<ReportResponse> getReport(
            @PathVariable Long reportId
    ) {
        return ApiResponse.ok(reportService.getReport(reportId));
    }

    @GetMapping("/api/v1/reports")
    public ApiResponse<List<ReportListResponse>> getReportList() {
        return ApiResponse.ok(reportService.getReportList());
    }


}

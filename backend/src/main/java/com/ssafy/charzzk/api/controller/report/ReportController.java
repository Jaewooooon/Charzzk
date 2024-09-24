package com.ssafy.charzzk.api.controller.report;

import com.ssafy.charzzk.api.ApiResponse;
import com.ssafy.charzzk.api.controller.car.request.CarRequest;
import com.ssafy.charzzk.api.controller.report.request.ReportRequest;
import com.ssafy.charzzk.api.service.car.response.CarResponse;
import com.ssafy.charzzk.api.service.report.ReportService;
import com.ssafy.charzzk.api.service.report.S3ImageService;
import com.ssafy.charzzk.api.service.report.response.ReportResponse;
import com.ssafy.charzzk.core.annotation.CurrentUser;
import com.ssafy.charzzk.domain.user.User;
import jakarta.validation.Valid;
import lombok.RequiredArgsConstructor;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestPart;
import org.springframework.web.bind.annotation.RestController;
import org.springframework.web.multipart.MultipartFile;

@RequiredArgsConstructor
@RestController
public class ReportController {

    private final ReportService reportService;
    private final S3ImageService s3ImageService;

    @PostMapping("/api/v1/reports")
    public ApiResponse<ReportResponse> createReport(
            @CurrentUser User user,
            @RequestPart("report") @Valid  ReportRequest request,
            @RequestPart(value = "image", required = false) MultipartFile image
    ) {
        String imageUrl = null;
        if (image != null && !image.isEmpty()) {
            imageUrl = s3ImageService.upload(image);
        }

        Long reportId = reportService.createReport(user, request.toServiceRequest(imageUrl));
        return ApiResponse.ok(reportService.getReport(reportId));
    }
}

package com.ssafy.charzzk.api.service.report;

import com.ssafy.charzzk.api.service.report.request.ReportServiceRequest;
import com.ssafy.charzzk.api.service.report.response.ReportListResponse;
import com.ssafy.charzzk.api.service.report.response.ReportResponse;
import com.ssafy.charzzk.core.exception.BaseException;
import com.ssafy.charzzk.core.exception.ErrorCode;
import com.ssafy.charzzk.domain.charger.Charger;
import com.ssafy.charzzk.domain.charger.ChargerRepository;
import com.ssafy.charzzk.domain.parkinglot.ParkingLot;
import com.ssafy.charzzk.domain.report.Report;
import com.ssafy.charzzk.domain.report.ReportRepository;
import com.ssafy.charzzk.domain.user.User;
import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;
import org.springframework.web.multipart.MultipartFile;

import java.util.List;
import java.util.stream.Collectors;

@Transactional(readOnly = true)
@RequiredArgsConstructor
@Service
public class ReportService {

    private final ReportRepository reportRepository;
    private final ChargerRepository chargerRepository;
    private final S3ImageService s3ImageService;

    @Transactional
    public Long createReport(User user, ReportServiceRequest serviceRequest, MultipartFile image) {
        // 주차장 찾기
        Charger charger = chargerRepository.findBySerialNumber(serviceRequest.getSerialNumber())
                .orElseThrow(() -> new BaseException(ErrorCode.CHARGER_NOT_FOUND));
        ParkingLot parkingLot = charger.getParkingLot();

        // 이미지가 있다면 이미지 업로드
        String imageUrl = null;
        if (image != null && !image.isEmpty()) {
            imageUrl = s3ImageService.upload(image);
        }

        Report report = Report.create(user, parkingLot, serviceRequest.getType(), serviceRequest.getContent(), imageUrl);
        reportRepository.save(report);
        return report.getId();
    }

    public ReportResponse getReport(Long reportId) {
        Report findReport = reportRepository.findByIdWithUserAndParkingLot(reportId).orElseThrow(
                () -> new BaseException(ErrorCode.REPORT_NOT_FOUND)
        );
        return ReportResponse.from(findReport);
    }

    @Transactional
    public void readReport(User user, Long reportId) {
        Report report = reportRepository.findByIdWithUserAndParkingLot(reportId).orElseThrow(
                () -> new BaseException(ErrorCode.REPORT_NOT_FOUND)
        );
        // TODO: 관리자가 맞는지 검증

        // 이미 확인된 신고는 다시 처리하지 않음
        if (!report.isRead()) {
            report.readReport();
        }
    }

    public List<ReportListResponse> getReportList() {
        List<Report> reportList = reportRepository.findAllWithUserAndParkingLot();
        return reportList.stream()
                .map(ReportListResponse::from)
                .collect(Collectors.toList());
    }
}

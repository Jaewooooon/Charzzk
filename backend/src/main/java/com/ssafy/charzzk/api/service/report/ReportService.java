package com.ssafy.charzzk.api.service.report;

import com.ssafy.charzzk.api.service.report.request.ReportServiceRequest;
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

@Transactional(readOnly = true)
@RequiredArgsConstructor
@Service
public class ReportService {

    private final ReportRepository reportRepository;
    private final ChargerRepository chargerRepository;

    @Transactional
    public Long createReport(User user, ReportServiceRequest serviceRequest) {
        // 주차장 찾기
        Charger charger = chargerRepository.findBySerialNumber(serviceRequest.getSerialNumber())
                .orElseThrow(() -> new BaseException(ErrorCode.CHARGER_NOT_FOUND));
        ParkingLot parkingLot = charger.getParkingLot();

        Report report = Report.create(user, parkingLot, serviceRequest.getType(), serviceRequest.getContent(), serviceRequest.getImage());
        reportRepository.save(report);
        return report.getId();
    }

    public ReportResponse getReport(Long reportId) {
        Report findReport = reportRepository.findById(reportId).orElseThrow(
                () -> new BaseException(ErrorCode.REPORT_NOT_FOUND)
        );
        return ReportResponse.from(findReport);
    }
}

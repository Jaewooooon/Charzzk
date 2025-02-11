package com.ssafy.charzzk.api.controller.report.request;

import com.ssafy.charzzk.api.service.report.request.ReportServiceRequest;
import com.ssafy.charzzk.domain.report.ReportType;
import jakarta.validation.constraints.NotNull;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Getter
@NoArgsConstructor
public class ReportRequest {

    private String serialNumber;

    @NotNull(message = "신고 유형은 필수입니다.")
    private ReportType type;

    @NotNull(message = "신고 내용은 필수입니다.")
    private String content;

    @Builder
    private ReportRequest(String serialNumber, ReportType type, String content) {
        this.serialNumber = serialNumber;
        this.type = type;
        this.content = content;
    }

    public ReportServiceRequest toServiceRequest() {
        return ReportServiceRequest.builder()
                .serialNumber(serialNumber)
                .type(type)
                .content(content)
                .build();
    }
}

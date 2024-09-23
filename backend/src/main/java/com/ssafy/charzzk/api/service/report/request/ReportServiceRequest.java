package com.ssafy.charzzk.api.service.report.request;

import com.ssafy.charzzk.domain.report.ReportType;
import jakarta.validation.constraints.NotNull;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Getter
@NoArgsConstructor
public class ReportServiceRequest {

    private String serialNumber;

    @NotNull(message = "신고 유형은 필수입니다.")
    private ReportType type;

    @NotNull(message = "신고 내용은 필수입니다.")
    private String content;

    private String image;

    @Builder
    private ReportServiceRequest(String serialNumber, ReportType type, String content, String image) {
        this.serialNumber = serialNumber;
        this.type = type;
        this.content = content;
        this.image = image;
    }
}

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

    private ReportType type;

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

package com.ssafy.charzzk.api.service.response;

import com.fasterxml.jackson.annotation.JsonFormat;
import lombok.Builder;
import lombok.Getter;

import java.time.LocalDateTime;

@Getter
public class ReservationCheckTimeResponse {

    private Long chargerId;
    private boolean isPossibleNow;
    @JsonFormat(shape = JsonFormat.Shape.STRING, pattern = "yyyy-MM-dd'T'HH:mm:ss")
    private LocalDateTime startTime;
    @JsonFormat(shape = JsonFormat.Shape.STRING, pattern = "yyyy-MM-dd'T'HH:mm:ss")
    private LocalDateTime endTime;

    @Builder
    public ReservationCheckTimeResponse(Long chargerId, boolean isPossibleNow, LocalDateTime startTime, LocalDateTime endTime) {
        this.chargerId = chargerId;
        this.isPossibleNow = isPossibleNow;
        this.startTime = startTime;
        this.endTime = endTime;
    }

}

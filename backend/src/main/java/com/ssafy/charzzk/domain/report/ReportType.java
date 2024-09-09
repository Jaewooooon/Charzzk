package com.ssafy.charzzk.domain.report;

public enum ReportType {
    FLIPPED,        // 로봇이 뒤집혀 있는 경우
    BROKEN,         // 로봇이 고장난 경우 (미작동, 충전 불가 등)
    DAMAGED,        // 로봇의 외관이 손상된 경우 (부품이 빠졌다거나...)
    ETC;            // 기타 신고
}

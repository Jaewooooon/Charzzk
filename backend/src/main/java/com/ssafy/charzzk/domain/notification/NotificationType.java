package com.ssafy.charzzk.domain.notification;

public enum NotificationType {
    CHARGING_STARTED,       // 충전이 시작되었을 때
//    CHARGING_IN_PROGRESS,   // 충전 중일 때 (상태 업데이트 용) -> 굳이 알림 필요없을 수도
    CHARGING_COMPLETED,     // 충전이 완료되었을 때
    CHARGING_FAILED,        // 충전이 실패했을 때 (에러, 로봇 배터리부족 등?)
    CHARGING_CANCELLED,     // 충전이 취소되었을 때 (사용자의 요청으로)
    PAYMENT_COMPLETED;      // 충전 후 결제가 완료되었을 때
}

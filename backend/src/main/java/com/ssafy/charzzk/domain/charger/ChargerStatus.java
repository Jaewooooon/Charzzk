package com.ssafy.charzzk.domain.charger;

import java.util.List;

public enum ChargerStatus {
    WAITING,            // 로봇이 대기 중인 상태
    MOVING_TO_CAR,      // 충전할 차량으로 이동 중인 상태
    RETURNING_TO_CAR,   // 차량 충전 후 출발 위치로 돌아가는 상태
    CAR_CHARGING,       // 차량 충전 중인 상태
    CHARGER_CHARGING,   // 충전 로봇 자체를 충전 중인 상태
//    COMPLETED,          // 충전 완료된 상태 -> 근데 어차피 RETURNING_TO_CAR와 겹치는 거 같음
    ERROR,              // 충전 로봇 에러 발생
    STOP,                // 관리자에 의한 중단 명령
    MAINTENANCE;         // 충전 로봇 점검 및 유지보수 상태

    public boolean isAvailable() {
        return List.of(WAITING, MOVING_TO_CAR, RETURNING_TO_CAR, CAR_CHARGING, CHARGER_CHARGING).contains(this);
    }
}

package com.ssafy.charzzk.core.exception;

import lombok.AllArgsConstructor;
import lombok.Getter;
import org.springframework.http.HttpStatus;

@Getter
@AllArgsConstructor
public enum ErrorCode {
    // Common
    SERVER_ERROR(1000, HttpStatus.INTERNAL_SERVER_ERROR, "서버 에러가 발생하였습니다."),

    // Authentication, Authorization
    UNAUTHORIZED(2000, HttpStatus.UNAUTHORIZED, "인증되지 않은 사용자입니다."),
    FORBIDDEN(2001, HttpStatus.FORBIDDEN, "권한이 없습니다."),
    INVALID_TOKEN(2002, HttpStatus.UNAUTHORIZED, "유효하지 않은 토큰입니다."),
    EXPIRED_TOKEN(2003, HttpStatus.UNAUTHORIZED, "만료된 토큰입니다."),
    INVALID_REFRESH_TOKEN(2004, HttpStatus.UNAUTHORIZED, "유효하지 않은 리프레시 토큰입니다."),
    EXPIRED_REFRESH_TOKEN(2005, HttpStatus.UNAUTHORIZED, "만료된 리프레시 토큰입니다."),
    ALREADY_AUTHORIZED(2006, HttpStatus.BAD_REQUEST, "이미 인증된 사용자입니다."),
    NOT_FOUND_USER(2007, HttpStatus.NOT_FOUND, "사용자를 찾을 수 없습니다."),
    INVALID_ACCESS_TOKEN(2008, HttpStatus.UNAUTHORIZED, "유효하지 않은 액세스 토큰입니다."),
    INVALID_MATTERMOST_INFO(2009, HttpStatus.UNAUTHORIZED, "메타모스트 정보가 일치하지 않습니다."),
    ACCESS_DENIED(2010, HttpStatus.FORBIDDEN, "접근 권한이 없습니다."),
    NICKNAME_ALREADY_EXISTS(2011, HttpStatus.BAD_REQUEST, "이미 존재하는 사용자 이름입니다."),

    // Car, CarType
    CAR_NOT_FOUND(3000, HttpStatus.NOT_FOUND, "차량을 찾을 수 없습니다."),
    CAR_TYPE_NOT_FOUND(3001, HttpStatus.NOT_FOUND, "차 기종을 찾을 수 없습니다."),
    CAR_NUMBER_ALREADY_EXISTS(3002, HttpStatus.BAD_REQUEST, "이미 존재하는 차량 번호입니다."),
    CAR_NOT_BELONG_TO_USER(3004, HttpStatus.FORBIDDEN, "해당 차량은 사용자의 차량이 아닙니다."),

    // Report
    REPORT_NOT_FOUND(4000, HttpStatus.NOT_FOUND, "신고를 찾을 수 없습니다."),
    EMPTY_FILE_EXCEPTION(4001, HttpStatus.BAD_REQUEST, "파일이 유효하지 않습니다."),
    NO_FILE_EXTENSION(4002, HttpStatus.BAD_REQUEST, "파일 확장자 명이 없습니다."),
    IO_EXCEPTION_ON_IMAGE_UPLOAD(4003, HttpStatus.INTERNAL_SERVER_ERROR, "이미지 업로드 중 입출력 오류가 발생했습니다."),
    INVALID_FILE_EXTENSION(4004, HttpStatus.BAD_REQUEST, "유효하지 않은 확장자 명입니다."),
    PUT_OBJECT_EXCEPTION(4005, HttpStatus.INTERNAL_SERVER_ERROR, "S3에 파일 업로드 중 오류가 발생했습니다."),
    FILE_SIZE_EXCEEDS_LIMIT(4006, HttpStatus.BAD_REQUEST, "파일 크기가 제한을 초과했습니다. (최대 1MB)"),

    // Charger
    CHARGER_NOT_FOUND(5000, HttpStatus.NOT_FOUND, "충전 로봇을 찾을 수 없습니다."),
    CHARGER_NOT_EXIST_IN_PARKING_LOT(5001, HttpStatus.NOT_FOUND, "주차장에 충전기가 없습니다."),
    NO_AVAILABLE_CHARGER(5002, HttpStatus.BAD_REQUEST, "이용 가능한 충전기가 없습니다."),

    // Reservation
    RESERVATION_NOT_FOUND(6000, HttpStatus.NOT_FOUND, "예약을 찾을 수 없습니다."),
    RESERVATION_CONFIRM_TIMEOUT(6001, HttpStatus.BAD_REQUEST, "예약 확정 시간이 초과되었습니다."),
    CANNOT_CANCEL_RESERVATION(6002, HttpStatus.BAD_REQUEST, "예약을 취소할 수 없습니다."),

    // Parking
    PARKING_SPOT_NOT_FOUND(7001, HttpStatus.NOT_FOUND, "주차 공간을 찾을 수 없습니다."),
    PARKING_LOT_NOT_FOUND(7002, HttpStatus.NOT_FOUND, "주차장이 존재하지 않습니다."),

    API_REQUEST_FAILED(8000, HttpStatus.BAD_REQUEST, "API 요청에 실패하였습니다."),

    ;
    private final int code;
    private final HttpStatus status;
    private final String message;
}

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
	CAR_TYPE_NOT_FOUND(3001, HttpStatus.NOT_FOUND, "차 기종을 찾을 수 없습니다.");

	private final int code;
	private final HttpStatus status;
	private final String message;
}

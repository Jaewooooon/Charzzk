package com.ssafy.charzzk.core.apiclient.response;

import lombok.Builder;
import lombok.Getter;

@Getter
public class ChargerCommandResponse {

    private String status;
    private String message;
    private Object data;

    @Builder
    private ChargerCommandResponse(String status, String message, Object data) {
        this.status = status;
        this.message = message;
        this.data = data;
    }
}

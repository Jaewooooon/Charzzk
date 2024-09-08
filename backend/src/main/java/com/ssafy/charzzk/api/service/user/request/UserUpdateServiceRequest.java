package com.ssafy.charzzk.api.service.user.request;

import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Getter
@NoArgsConstructor
public class UserUpdateServiceRequest {

    private String nickname;

    @Builder
    private UserUpdateServiceRequest(String nickname) {
        this.nickname = nickname;
    }
}

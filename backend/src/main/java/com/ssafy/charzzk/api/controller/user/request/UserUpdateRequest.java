package com.ssafy.charzzk.api.controller.user.request;

import com.ssafy.charzzk.api.service.user.request.UserUpdateServiceRequest;
import jakarta.validation.constraints.NotBlank;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Getter
@NoArgsConstructor
public class UserUpdateRequest {

    @NotBlank(message = "닉네임은 필수입니다.")
    private String nickname;

    @Builder
    private UserUpdateRequest(String nickname) {
        this.nickname = nickname;
    }

    public UserUpdateServiceRequest toServiceRequest() {
        return UserUpdateServiceRequest.builder()
            .nickname(nickname)
            .build();
    }
}

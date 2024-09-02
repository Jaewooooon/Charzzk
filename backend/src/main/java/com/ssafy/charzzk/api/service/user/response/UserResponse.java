package com.ssafy.charzzk.api.service.user.response;

import com.ssafy.charzzk.domain.user.User;
import lombok.Builder;
import lombok.Getter;

@Getter
public class UserResponse {

    private Long id;
    private String username;

    @Builder
    public UserResponse(Long id, String username) {
        this.id = id;
        this.username = username;
    }

    public static UserResponse of(User user) {
        return UserResponse.builder()
                .id(user.getId())
                .username(user.getUsername())
                .build();
    }
}

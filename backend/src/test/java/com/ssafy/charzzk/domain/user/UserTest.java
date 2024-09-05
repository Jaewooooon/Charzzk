package com.ssafy.charzzk.domain.user;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class UserTest {

    @DisplayName("유저의 닉네임을 변경한다")
    @Test
    public void updateNickname() {
        User user = User.builder()
                .username("test@gmail.com")
                .nickname("nickname")
                .build();

        user.updateNickname("newNickname");

        assertEquals(user.getNickname(), "newNickname");
    }
}

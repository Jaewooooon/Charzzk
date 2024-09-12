package com.ssafy.charzzk.domain.user.service;

import com.ssafy.charzzk.IntegrationTestSupport;
import com.ssafy.charzzk.api.service.user.UserService;
import com.ssafy.charzzk.api.service.user.request.UserUpdateServiceRequest;
import com.ssafy.charzzk.api.service.user.response.UserResponse;
import com.ssafy.charzzk.core.exception.BaseException;
import com.ssafy.charzzk.core.exception.ErrorCode;
import com.ssafy.charzzk.domain.user.User;
import com.ssafy.charzzk.domain.user.UserRepository;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.test.context.ActiveProfiles;

import java.util.List;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;


@ActiveProfiles("test")
class UserServiceTest extends IntegrationTestSupport {
    @Autowired
    private UserService userService;

    @Autowired
    private UserRepository userRepository;

    @AfterEach
    void tearDown() {
        userRepository.deleteAllInBatch();
    }

    @DisplayName("로그인한 유저를 조회한다")
    @Test
    public void getUser() {
        User user = User.builder()
                .username("test@gmail.com")
                .nickname("nickname")
                .build();
        userRepository.save(user);

        UserResponse response = userService.getUser(user);

        assertThat(response)
                .extracting("username", "nickname")
                .contains("test@gmail.com", "nickname");
    }

    @DisplayName("없는 유저를 조회한다")
    @Test
    public void getNotExistUser() {
        User user = User.builder()
                .username("test@gmail.com")
                .nickname("nickname")
                .build();

        assertThatThrownBy(() -> userService.getUser(user))
                .isInstanceOf(BaseException.class)
                .hasMessage(ErrorCode.NOT_FOUND_USER.getMessage());
    }

    @DisplayName("유저가 닉네임을 수정하면 수정된 닉네임을 반환한다.")
    @Test
    public void updateNickname() {
        // given
        User user = User.builder()
                .username("test@gmail.com")
                .nickname("nickname")
                .build();

        UserUpdateServiceRequest request = UserUpdateServiceRequest.builder()
                .nickname("newNickname")
                .build();

        userRepository.save(user);

        // when
        userService.updateNickname(user, request);

        // then
        assertThat(user.getNickname()).isEqualTo("newNickname");
    }

    @DisplayName("이미 존재하는 닉네임으로 수정하려고 하면 예외가 발생한다.")
    @Test
    public void updateUsernameWithExistingUsername() {
        User user1 = User.builder()
                .username("test1@gmail.com")
                .nickname("nickname1")
                .build();

        User user2 = User.builder()
                .username("test2@gmail.com")
                .nickname("nickname2")
                .build();

        userRepository.saveAll(List.of(user1, user2));

        UserUpdateServiceRequest request = UserUpdateServiceRequest.builder()
                .nickname("nickname1")
                .build();

        assertThatThrownBy(() -> userService.updateNickname(user2, request))
                .isInstanceOf(BaseException.class)
                .hasMessage(ErrorCode.NICKNAME_ALREADY_EXISTS.getMessage());
    }
}

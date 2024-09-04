package com.ssafy.charzzk.domain.user.service;

import com.ssafy.charzzk.IntegrationTestSupport;
import com.ssafy.charzzk.api.service.user.UserService;
import com.ssafy.charzzk.api.service.user.response.UserResponse;
import com.ssafy.charzzk.domain.user.User;
import com.ssafy.charzzk.domain.user.UserRepository;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;

import java.util.List;

import static org.assertj.core.api.Assertions.assertThat;


class UserServiceTest extends IntegrationTestSupport {
    @Autowired
    private UserService userService;

    @Autowired
    private UserRepository userRepository;

    @AfterEach
    void tearDown() {
        userRepository.deleteAllInBatch();
    }

//    @DisplayName("유저 목록 조회")
//    @Test
//    void getUserList() {
//        // given
//        userRepository.saveAll(List.of(
//                User.create("test_user1"),
//                User.create("test_user2"),
//                User.create("test_user3")
//        ));
//
//        // when
//        List<UserResponse> userResponse = userService.getUsers();
//
//        // then
//        assertThat(userResponse.size()).isEqualTo(3);
//        assertThat(userResponse)
//                .extracting("username")
//                .containsExactlyInAnyOrder("test_user1", "test_user2", "test_user3");
//
//    }
}

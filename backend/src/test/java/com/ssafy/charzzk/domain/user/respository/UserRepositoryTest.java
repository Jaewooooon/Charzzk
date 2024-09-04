package com.ssafy.charzzk.domain.user.respository;

import com.ssafy.charzzk.IntegrationTestSupport;
import com.ssafy.charzzk.domain.user.User;
import com.ssafy.charzzk.domain.user.UserRepository;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.transaction.annotation.Transactional;

import java.util.List;

import static org.assertj.core.api.Assertions.assertThat;

@Transactional
class UserRepositoryTest extends IntegrationTestSupport {

    @Autowired
    private UserRepository userRepository;

//    @DisplayName("유저 목록 조회")
//    @Test
//    void getUserList() {
//        // given
//        List<User> users = List.of(
//                User.create("test_user1"),
//                User.create("test_user2"),
//                User.create("test_user3")
//        );
//        userRepository.saveAll(users);
//
//        // when
//        List<User> findUsers = userRepository.findAll();
//
//        // then
//        assertThat(findUsers.size()).isEqualTo(3);
//        assertThat(findUsers)
//                .extracting("username")
//                .containsExactlyInAnyOrder("test_user1", "test_user2", "test_user3");
//    }
}

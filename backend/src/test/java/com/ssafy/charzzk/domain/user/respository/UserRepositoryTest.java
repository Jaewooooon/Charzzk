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




}

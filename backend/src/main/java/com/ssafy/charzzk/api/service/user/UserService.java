package com.ssafy.charzzk.api.service.user;

import com.ssafy.charzzk.api.service.user.request.UserUpdateServiceRequest;
import com.ssafy.charzzk.api.service.user.response.UserResponse;
import com.ssafy.charzzk.domain.user.User;
import com.ssafy.charzzk.domain.user.UserRepository;
import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

import java.util.List;

@Transactional(readOnly = true)
@RequiredArgsConstructor
@Service
public class UserService {
    private final UserRepository userRepository;

    public List<UserResponse> getUsers() {
        List<User> users = userRepository.findAll();

        return users.stream().map(UserResponse::of).toList();
    }

    public UserResponse getUser(User user) {
        return null;
    }

    @Transactional
    public void updateNickname(User user, UserUpdateServiceRequest request) {
    }
}

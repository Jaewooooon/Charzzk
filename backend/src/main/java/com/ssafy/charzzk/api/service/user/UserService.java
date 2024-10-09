package com.ssafy.charzzk.api.service.user;

import com.ssafy.charzzk.api.service.user.request.UserUpdateServiceRequest;
import com.ssafy.charzzk.api.service.user.response.UserResponse;
import com.ssafy.charzzk.core.exception.BaseException;
import com.ssafy.charzzk.core.exception.ErrorCode;
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

    public UserResponse getUser(User user) {
        User findUser = userRepository.findByUsername(user.getUsername()).orElseThrow(
                () -> new BaseException(ErrorCode.NOT_FOUND_USER));

        return UserResponse.of(findUser);
    }

    @Transactional
    public void updateNickname(User user, UserUpdateServiceRequest request) {
        if (userRepository.existsByNickname(request.getNickname())) {
            throw new BaseException(ErrorCode.NICKNAME_ALREADY_EXISTS);
        }

        user.updateNickname(request.getNickname());
        userRepository.save(user);
    }

    public String checkNickname(String nickname) {
        if (userRepository.existsByNickname(nickname)) {
            throw new BaseException(ErrorCode.NICKNAME_ALREADY_EXISTS);
        }

        return "닉네임 변경이 가능합니다";
    }
}

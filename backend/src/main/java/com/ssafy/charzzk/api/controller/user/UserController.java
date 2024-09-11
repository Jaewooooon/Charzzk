package com.ssafy.charzzk.api.controller.user;

import com.ssafy.charzzk.api.ApiResponse;
import com.ssafy.charzzk.api.controller.user.request.UserUpdateRequest;
import com.ssafy.charzzk.api.service.user.UserService;
import com.ssafy.charzzk.api.service.user.response.UserResponse;
import com.ssafy.charzzk.core.annotation.CurrentUser;
import com.ssafy.charzzk.domain.user.User;
import jakarta.validation.Valid;
import lombok.RequiredArgsConstructor;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PatchMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RestController;

import java.util.List;

@RequiredArgsConstructor
@RestController
public class UserController {
    private final UserService userService;

    @GetMapping(value = "/api/v1/users")
    public ApiResponse<List<UserResponse>> getUsers() {
        return ApiResponse.ok(userService.getUsers());
    }

    @PatchMapping(value = "/api/v1/users/nickname")
    public ApiResponse<UserResponse> updateNickname(
            @CurrentUser User user,
            @Valid @RequestBody UserUpdateRequest request
    ) {
        userService.updateNickname(user, request.toServiceRequest());
        return ApiResponse.ok(userService.getUser(user));
    }
}

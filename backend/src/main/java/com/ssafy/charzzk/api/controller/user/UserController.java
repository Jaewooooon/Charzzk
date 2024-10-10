package com.ssafy.charzzk.api.controller.user;

import com.ssafy.charzzk.api.ApiResponse;
import com.ssafy.charzzk.api.controller.user.request.UserUpdateRequest;
import com.ssafy.charzzk.api.service.user.UserService;
import com.ssafy.charzzk.api.service.user.response.UserResponse;
import com.ssafy.charzzk.core.annotation.CurrentUser;
import com.ssafy.charzzk.domain.user.User;
import jakarta.validation.Valid;
import jakarta.validation.constraints.NotBlank;
import lombok.RequiredArgsConstructor;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PatchMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestParam;
import org.springframework.web.bind.annotation.RestController;

@RequiredArgsConstructor
@RestController
public class UserController {
    private final UserService userService;

    @GetMapping(value = "/api/v1/users/me")
    public ApiResponse<UserResponse> getUser(
            @CurrentUser User user
    ) {
        return ApiResponse.ok(userService.getUser(user));
    }

    @GetMapping(value = "/api/v1/users/check-nickname")
    public ApiResponse<String> checkNickname(
            @RequestParam @NotBlank String nickname
    ) {
        return ApiResponse.ok(userService.checkNickname(nickname));
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

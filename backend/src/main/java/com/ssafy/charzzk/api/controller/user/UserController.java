package com.ssafy.charzzk.api.controller.user;

import com.ssafy.charzzk.api.ApiResponse;
import com.ssafy.charzzk.api.service.user.UserService;
import com.ssafy.charzzk.api.service.user.response.UserResponse;
import lombok.RequiredArgsConstructor;
import org.springframework.web.bind.annotation.GetMapping;
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
}

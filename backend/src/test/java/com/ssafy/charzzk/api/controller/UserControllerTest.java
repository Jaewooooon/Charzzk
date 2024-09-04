package com.ssafy.charzzk.api.controller;

import com.ssafy.charzzk.ControllerTestSupport;
import com.ssafy.charzzk.api.controller.user.request.UserUpdateRequest;
import com.ssafy.charzzk.api.service.user.request.UserUpdateServiceRequest;
import com.ssafy.charzzk.api.service.user.response.UserResponse;
import com.ssafy.charzzk.domain.user.User;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.springframework.http.MediaType;
import org.springframework.security.test.context.support.WithMockUser;
import org.springframework.test.web.servlet.ResultActions;

import static org.mockito.ArgumentMatchers.any;
import static org.mockito.BDDMockito.given;
import static org.springframework.security.test.web.servlet.request.SecurityMockMvcRequestPostProcessors.csrf;
import static org.springframework.test.web.servlet.request.MockMvcRequestBuilders.get;
import static org.springframework.test.web.servlet.request.MockMvcRequestBuilders.patch;
import static org.springframework.test.web.servlet.result.MockMvcResultHandlers.print;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.jsonPath;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.status;

class UserControllerTest extends ControllerTestSupport {

    @DisplayName("유저 목록을 조회한다.")
    @WithMockUser
    @Test
    void getUsers() throws Exception {
        // given

        // when
        ResultActions perform = mockMvc.perform(
                get("/api/v1/users")
        );

        // then
        perform.andDo(print())
                .andExpect(status().isOk())
                .andExpect(jsonPath("$.code").value("200"))
                .andExpect(jsonPath("$.status").value("OK"))
                .andExpect(jsonPath("$.message").value("OK"))
                .andExpect(jsonPath("$.data").isArray());
    }

    @DisplayName("유저가 올바른 닉네임으로 닉네임을 수정하면 성공한다.")
    @WithMockUser
    @Test
    public void changeUserNickname() throws Exception {
        // given
        UserUpdateRequest request = UserUpdateRequest.builder()
                .nickname("New nickname")
                .build();

        UserResponse response = UserResponse.builder()
                .id(1L)
                .nickname("New nickname")
                .build();

        given(userService.updateNickname(any(User.class), any(UserUpdateServiceRequest.class)))
                .willReturn(response);

        // when
        ResultActions perform = mockMvc.perform(
                patch("/api/v1/users/nickname")
                        .with(csrf())
                        .content(objectMapper.writeValueAsString(request))
                        .contentType(MediaType.APPLICATION_JSON)
        );

        // then
        perform.andDo(print())
                .andExpect(status().isOk())
                .andExpect(jsonPath("$.code").value("200"))
                .andExpect(jsonPath("$.status").value("OK"))
                .andExpect(jsonPath("$.message").value("OK"))
                .andExpect(jsonPath("$.data.id").value(1L))
                .andExpect(jsonPath("$.data.nickname").value("New nickname"));
    }

    @DisplayName("유저가 빈 닉네임으로 닉네임을 수정하면 실패한다.")
    @WithMockUser
    @Test
    public void changeUserNicknameEmpty() throws Exception {
        // given
        UserUpdateRequest request = UserUpdateRequest.builder()
                .nickname("")
                .build();

        // when
        ResultActions perform = mockMvc.perform(
                patch("/api/v1/users/nickname")
                        .with(csrf())
                        .content(objectMapper.writeValueAsString(request))
                        .contentType(MediaType.APPLICATION_JSON)
        );

        // then
        perform.andDo(print())
                .andExpect(status().isBadRequest())
                .andExpect(jsonPath("$.code").value("400"))
                .andExpect(jsonPath("$.status").value("BAD_REQUEST"))
                .andExpect(jsonPath("$.message").value("닉네임은 필수입니다."))
                .andExpect(jsonPath("$.data").isEmpty());
    }

}

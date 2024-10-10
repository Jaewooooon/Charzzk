package com.ssafy.charzzk.docs.user;

import com.epages.restdocs.apispec.ResourceSnippetParameters;
import com.ssafy.charzzk.api.controller.user.UserController;
import com.ssafy.charzzk.api.controller.user.request.UserUpdateRequest;
import com.ssafy.charzzk.api.service.user.UserService;
import com.ssafy.charzzk.api.service.user.request.UserUpdateServiceRequest;
import com.ssafy.charzzk.api.service.user.response.UserResponse;
import com.ssafy.charzzk.docs.RestDocsSupport;
import com.ssafy.charzzk.domain.user.User;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.springframework.http.MediaType;
import org.springframework.restdocs.payload.JsonFieldType;
import org.springframework.security.test.context.support.WithMockUser;
import org.springframework.test.web.servlet.ResultActions;
import org.springframework.test.web.servlet.request.MockMvcRequestBuilders;

import java.util.List;

import static com.epages.restdocs.apispec.MockMvcRestDocumentationWrapper.document;
import static com.epages.restdocs.apispec.ResourceDocumentation.resource;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.anyString;
import static org.mockito.BDDMockito.given;
import static org.mockito.Mockito.mock;
import static org.springframework.restdocs.mockmvc.RestDocumentationRequestBuilders.get;
import static org.springframework.restdocs.mockmvc.RestDocumentationRequestBuilders.patch;
import static org.springframework.restdocs.operation.preprocess.Preprocessors.preprocessResponse;
import static org.springframework.restdocs.operation.preprocess.Preprocessors.prettyPrint;
import static org.springframework.restdocs.payload.PayloadDocumentation.fieldWithPath;
import static org.springframework.restdocs.request.RequestDocumentation.parameterWithName;
import static org.springframework.security.test.web.servlet.request.SecurityMockMvcRequestPostProcessors.csrf;
import static org.springframework.test.web.servlet.result.MockMvcResultHandlers.print;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.jsonPath;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.status;

public class UserControllerDocsTest extends RestDocsSupport {

    private final UserService userService = mock(UserService.class);

    @Override
    protected Object initController() {
        return new UserController(userService);
    }

    @DisplayName("유저가 자신의 닉네을 조회한다.")
    @Test
    public void getUser() throws Exception {
        // given
        User user = User.builder()
                .id(1L)
                .username("username")
                .nickname("nickname")
                .build();

        given(userService.getUser(any(User.class)))
                .willReturn(UserResponse.of(user));

        // when
        ResultActions perform = mockMvc.perform(
                get("/api/v1/users/me")
                        .contentType(MediaType.APPLICATION_JSON)
        );

        // then
        perform
                .andDo(print())
                .andExpect(status().isOk())
                .andDo(document("get-user",
                        preprocessResponse(prettyPrint()),
                        resource(ResourceSnippetParameters.builder()
                                .tag("User")
                                .summary("유저 정보 확인")
                                .responseFields(
                                        fieldWithPath("code").type(JsonFieldType.NUMBER)
                                                .description("코드"),
                                        fieldWithPath("status").type(JsonFieldType.STRING)
                                                .description("상태"),
                                        fieldWithPath("message").type(JsonFieldType.STRING)
                                                .description("메시지"),
                                        fieldWithPath("data.id").type(JsonFieldType.NUMBER)
                                                .description("유저 아이디"),
                                        fieldWithPath("data.username").type(JsonFieldType.STRING)
                                                .description("로그인한 아이디"),
                                        fieldWithPath("data.nickname").type(JsonFieldType.STRING)
                                                .description("닉네임")
                                )
                                .build())));
    }


    @DisplayName("닉네임을 중복확인한다.")
    @Test
    void checkNickname() throws Exception {
        // given
        given(userService.checkNickname(anyString()))
                .willReturn("닉네임 변경이 가능합니다");

        // when
        ResultActions perform = mockMvc.perform(get("/api/v1/users/check-nickname")
                .param("nickname", "NewNickname")
                .contentType(MediaType.APPLICATION_JSON));

        // then
        perform
                .andDo(print())
                .andExpect(status().isOk())
                .andDo(document("check-nickname",
                        preprocessResponse(prettyPrint()),
                        resource(ResourceSnippetParameters.builder()
                                .tag("User")
                                .summary("닉네임 중복확인")
                                .queryParameters(
                                        parameterWithName("nickname").description("중복확인할 닉네임")
                                )
                                .responseFields(
                                        fieldWithPath("code").type(JsonFieldType.NUMBER)
                                                .description("코드"),
                                        fieldWithPath("status").type(JsonFieldType.STRING)
                                                .description("상태"),
                                        fieldWithPath("message").type(JsonFieldType.STRING)
                                                .description("메시지"),
                                        fieldWithPath("data").type(JsonFieldType.STRING)
                                                .description("성공 메시지")
                                )
                                .build())));
    }


    @DisplayName("유저 닉네임을 수정한다.")
    @Test
    void updateUserNickname() throws Exception {
        // given
        String nickname = "New nickname";

        UserUpdateRequest request = UserUpdateRequest.builder()
                .nickname(nickname)
                .build();

        UserResponse response = UserResponse.builder()
                .id(1L)
                .username("test@gmail.com")
                .nickname(nickname)
                .build();

        given(userService.getUser(any(User.class))).willReturn(response);

        // when, then
        mockMvc.perform(
                        patch("/api/v1/users/nickname")
                                .content(objectMapper.writeValueAsString(request))
                                .contentType(MediaType.APPLICATION_JSON)
                )
                .andDo(print())
                .andExpect(status().isOk())
                .andDo(document("user-update",
                        preprocessResponse(prettyPrint()),
                        resource(ResourceSnippetParameters.builder()
                                .tag("User")
                                .summary("유저 닉네임 업데이트")
                                .requestFields(
                                        fieldWithPath("nickname").type(JsonFieldType.STRING)
                                                .description("수정할 닉네임")
                                )
                                .responseFields(
                                        fieldWithPath("code").type(JsonFieldType.NUMBER)
                                                .description("코드"),
                                        fieldWithPath("status").type(JsonFieldType.STRING)
                                                .description("상태"),
                                        fieldWithPath("message").type(JsonFieldType.STRING)
                                                .description("메시지"),
                                        fieldWithPath("data.id").type(JsonFieldType.NUMBER)
                                                .description("아이디"),
                                        fieldWithPath("data.username").type(JsonFieldType.STRING)
                                                .description("유저 이름"),
                                        fieldWithPath("data.nickname").type(JsonFieldType.STRING)
                                                .description("닉네임")
                                )
                                .build())));
    }
}

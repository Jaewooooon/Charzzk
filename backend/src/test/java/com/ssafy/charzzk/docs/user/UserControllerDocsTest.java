package com.ssafy.charzzk.docs.user;

import com.ssafy.charzzk.api.controller.user.UserController;
import com.ssafy.charzzk.api.service.user.UserService;
import com.ssafy.charzzk.api.service.user.response.UserResponse;
import com.ssafy.charzzk.docs.RestDocsSupport;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.springframework.restdocs.payload.JsonFieldType;

import java.util.List;

import static org.mockito.BDDMockito.given;
import static org.mockito.Mockito.mock;
import static com.epages.restdocs.apispec.MockMvcRestDocumentationWrapper.document;
import static org.springframework.restdocs.operation.preprocess.Preprocessors.*;
import static org.springframework.restdocs.payload.PayloadDocumentation.fieldWithPath;
import static org.springframework.restdocs.payload.PayloadDocumentation.responseFields;
import static org.springframework.restdocs.mockmvc.RestDocumentationRequestBuilders.*;
import static org.springframework.test.web.servlet.result.MockMvcResultHandlers.print;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.status;

public class UserControllerDocsTest extends RestDocsSupport {

    private final UserService userService = mock(UserService.class);

    @Override
    protected Object initController() {
        return new UserController(userService);
    }

    @DisplayName("유저 목록을 조회한다.")
    @Test
    void getUsers() throws Exception {
        // given
        List<UserResponse> response = List.of(
                UserResponse.builder()
                        .id(1L)
                        .username("test")
                        .build()
        );

        given(userService.getUsers()).willReturn(response);

        // when, then
        mockMvc.perform(
                        get("/api/v1/users")
                )
                .andDo(print())
                .andExpect(status().isOk())
                .andDo(document("user-get",
                        preprocessResponse(prettyPrint()),
                        responseFields(
                                fieldWithPath("code").type(JsonFieldType.NUMBER)
                                        .description("코드"),
                                fieldWithPath("status").type(JsonFieldType.STRING)
                                        .description("상태"),
                                fieldWithPath("message").type(JsonFieldType.STRING)
                                        .description("메시지"),
                                fieldWithPath("data[].id").type(JsonFieldType.NUMBER)
                                        .description("아이디"),
                                fieldWithPath("data[].username").type(JsonFieldType.STRING)
                                        .description("유저 이름")
                                )
                        ));
    }
}

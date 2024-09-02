package com.ssafy.charzzk;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.ssafy.charzzk.api.service.auth.JWTService;
import com.ssafy.charzzk.api.controller.user.UserController;
import com.ssafy.charzzk.api.service.user.UserService;
import com.ssafy.charzzk.core.configuration.SecurityConfig;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.test.autoconfigure.web.servlet.WebMvcTest;
import org.springframework.boot.test.mock.mockito.MockBean;
import org.springframework.context.annotation.ComponentScan;
import org.springframework.context.annotation.FilterType;
import org.springframework.test.web.servlet.MockMvc;

@WebMvcTest(
        controllers = {UserController.class},
        excludeFilters = {
                @ComponentScan.Filter(type = FilterType.ASSIGNABLE_TYPE, classes = {SecurityConfig.class})
        }
)
public abstract class ControllerTestSupport {

    @Autowired
    protected MockMvc mockMvc;

    @Autowired
    protected ObjectMapper objectMapper;

    @MockBean
    protected UserService userService;

    @MockBean
    protected JWTService jwtService;
}

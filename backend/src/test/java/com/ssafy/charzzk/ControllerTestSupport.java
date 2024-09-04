package com.ssafy.charzzk;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.ssafy.charzzk.api.service.auth.CustomUserService;
import com.ssafy.charzzk.api.service.auth.JWTService;
import com.ssafy.charzzk.api.controller.user.UserController;
import com.ssafy.charzzk.api.service.user.UserService;
import com.ssafy.charzzk.core.configuration.SecurityConfig;
import com.ssafy.charzzk.core.filter.JWTFilter;
import com.ssafy.charzzk.domain.user.User;
import com.ssafy.charzzk.domain.user.UserRepository;
import jakarta.annotation.PostConstruct;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.test.autoconfigure.web.servlet.WebMvcTest;
import org.springframework.boot.test.context.TestConfiguration;
import org.springframework.boot.test.mock.mockito.MockBean;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.ComponentScan;
import org.springframework.context.annotation.FilterType;
import org.springframework.context.annotation.Import;
import org.springframework.test.web.servlet.MockMvc;

import java.util.function.Function;

import static org.mockito.BDDMockito.given;

@WebMvcTest(
        controllers = {UserController.class},
        excludeFilters = {
                @ComponentScan.Filter(type = FilterType.ASSIGNABLE_TYPE, classes = {SecurityConfig.class}),
                @ComponentScan.Filter(type = FilterType.ASSIGNABLE_TYPE, classes = {JWTFilter.class})
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


    @TestConfiguration
    static class TestConfig {
        @Bean
        public Function<Object, User> fetchUser() {
            return principal -> {
                // Implement test-specific logic or mock
                return null;
            };
        }
    }
}

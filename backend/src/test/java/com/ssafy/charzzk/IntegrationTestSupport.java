package com.ssafy.charzzk;

import com.ssafy.charzzk.api.service.auth.JWTService;
import com.ssafy.charzzk.container.TestContainer;
import com.ssafy.charzzk.domain.user.User;
import org.springframework.boot.autoconfigure.EnableAutoConfiguration;
import org.springframework.boot.autoconfigure.security.oauth2.client.servlet.OAuth2ClientAutoConfiguration;
import org.springframework.boot.autoconfigure.security.servlet.SecurityAutoConfiguration;
import org.springframework.boot.test.context.SpringBootTest;
import org.springframework.boot.test.mock.mockito.MockBean;
import org.springframework.data.redis.connection.RedisConnectionFactory;
import org.springframework.data.redis.core.RedisKeyValueAdapter;
import org.springframework.data.redis.core.RedisTemplate;
import org.springframework.data.redis.listener.RedisMessageListenerContainer;
import org.springframework.security.core.userdetails.UserDetails;

import java.util.function.Function;

@SpringBootTest
@EnableAutoConfiguration(exclude = {SecurityAutoConfiguration.class, OAuth2ClientAutoConfiguration.class})
public abstract class IntegrationTestSupport{

    @MockBean
    protected RedisConnectionFactory redisConnectionFactory;

    @MockBean
    protected RedisMessageListenerContainer redisMessageListener;

    @MockBean
    protected RedisTemplate<String, Object> redisTemplate;

    @MockBean
    protected RedisKeyValueAdapter redisKeyValueAdapter;

    @MockBean
    protected JWTService jwtService;

    @MockBean
    protected Function<UserDetails, User> fetchUser;
}

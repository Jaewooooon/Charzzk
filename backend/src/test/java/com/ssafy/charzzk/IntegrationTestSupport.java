package com.ssafy.charzzk;

import com.ssafy.charzzk.api.service.auth.JWTService;
import com.ssafy.charzzk.container.TestContainer;
import com.ssafy.charzzk.domain.user.User;
import org.springframework.boot.test.context.SpringBootTest;
import org.springframework.boot.test.mock.mockito.MockBean;
import org.springframework.data.redis.connection.RedisConnectionFactory;
import org.springframework.data.redis.core.RedisKeyValueAdapter;
import org.springframework.data.redis.core.RedisTemplate;
import org.springframework.data.redis.listener.RedisMessageListenerContainer;
import org.springframework.security.core.userdetails.UserDetails;

import java.util.function.Function;

@SpringBootTest(properties = {
        "spring.data.redis.host=localhost",
        "spring.data.redis.port=6379",
        "spring.data.redis.password=yourpassword"
})
public abstract class IntegrationTestSupport extends TestContainer {

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

package com.ssafy.charzzk.core.util;

import com.ssafy.charzzk.api.service.reservation.ReservationService;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import org.springframework.data.redis.connection.Message;

import static org.mockito.ArgumentMatchers.anyLong;
import static org.mockito.BDDMockito.given;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;

@ExtendWith(MockitoExtension.class)
class RedisKeyExpirationListenerTest {
    @Mock
    private ReservationService reservationService;

    @InjectMocks
    private RedisKeyExpirationListener redisKeyExpirationListener;

    @DisplayName("키 만료 시 timeout함수를 호출한다.")
    @Test
    void onMessageShouldCallTimeout() {
        // given
        Long reservationId = 1L;
        String expiredKey = "reservation:" + reservationId;

        Message message = mock(Message.class);
        given(message.getBody()).willReturn(expiredKey.getBytes());

        // when
        redisKeyExpirationListener.onMessage(message, null);

        // then
        verify(reservationService, times(1)).timeout(reservationId);
    }

    @DisplayName("만료된 키가 올바르지 않은 경우 예외가 발생하지 않는다.")
    @Test
    void onMessageShouldNotThrowExceptionForInvalidKey() {
        // given
        String expiredKey = "reservation:1:3";
        Message message = mock(Message.class);
        given(message.getBody()).willReturn(expiredKey.getBytes());

        // when
        redisKeyExpirationListener.onMessage(message, null);

        // then
        verify(reservationService).timeout(anyLong());
    }
}

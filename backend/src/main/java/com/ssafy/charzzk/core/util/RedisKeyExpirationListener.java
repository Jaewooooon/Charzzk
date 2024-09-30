package com.ssafy.charzzk.core.util;

import com.ssafy.charzzk.api.service.reservation.ReservationService;
import com.ssafy.charzzk.domain.reservation.ReservationRepository;
import lombok.RequiredArgsConstructor;
import org.springframework.data.redis.connection.Message;
import org.springframework.data.redis.connection.MessageListener;
import org.springframework.stereotype.Component;

@RequiredArgsConstructor
@Component
public class RedisKeyExpirationListener implements MessageListener {

    private final ReservationService reservationService;

    @Override
    public void onMessage(Message message, byte[] pattern) {
        String expiredKey = new String(message.getBody());

        performActionOnExpiration(expiredKey);
    }

    private void performActionOnExpiration(String expiredKey) {
        String[] ids = expiredKey.split(":");
        Long reservationId = Long.valueOf(ids[1]);

        reservationService.timeout(reservationId);
    }
}

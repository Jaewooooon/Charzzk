package com.ssafy.charzzk.domain.reservation;

import jakarta.annotation.PostConstruct;
import lombok.RequiredArgsConstructor;
import org.springframework.data.redis.core.RedisTemplate;
import org.springframework.data.redis.core.ValueOperations;
import org.springframework.stereotype.Repository;

import java.util.Set;
import java.util.concurrent.TimeUnit;

@RequiredArgsConstructor
@Repository
public class ReservationCacheRepository {

    public static final String RESERVATION_CACHE = "reservation:";

    private final RedisTemplate<String, String> redisTemplate;
    private ValueOperations<String, String> vop;

    @PostConstruct
    private void init() {
        vop = redisTemplate.opsForValue();
    }

    public void createReservationGracePeriod(Long reservationId, Long chargerId, int TTlSeconds) {
        String key = RESERVATION_CACHE + reservationId + ":" + chargerId;
        vop.set(key, "true", TTlSeconds, TimeUnit.SECONDS);
    }

    public void deleteAllReservations() {
        Set<String> keys = redisTemplate.keys(RESERVATION_CACHE + "*");
        if (keys != null && !keys.isEmpty()) {
            redisTemplate.delete(keys);
        }
    }

    public boolean existsReservationGracePeriod(Long reservationId, Long chargerId) {
        String key = RESERVATION_CACHE + reservationId + ":" + chargerId;

        if (vop.get(key) != null) {
            redisTemplate.delete(key);
            return true;
        }

        return false;
    }

}

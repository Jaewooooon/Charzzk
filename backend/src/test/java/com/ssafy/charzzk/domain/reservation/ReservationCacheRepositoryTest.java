package com.ssafy.charzzk.domain.reservation;

import com.ssafy.charzzk.IntegrationTestSupport;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.data.redis.core.RedisTemplate;
import org.springframework.test.context.ActiveProfiles;
import org.springframework.transaction.annotation.Transactional;

import static org.assertj.core.api.Assertions.assertThat;


@Transactional
@ActiveProfiles("test")
class ReservationCacheRepositoryTest extends IntegrationTestSupport {

    @Autowired
    private ReservationCacheRepository reservationCacheRepository;

    @Autowired
    private RedisTemplate<String, String> redisTemplate;

    @AfterEach
    public void tearDown() {
        reservationCacheRepository.deleteAllReservations();
    }

    @DisplayName("예약 아이디, 충전기 아이디로 구성된 키로 예약을 생성한다.")
    @Test
    public void createReservationGracePeriod() {
        // given
        Long reservationId = 1L;
        Long chargerId = 1L;
        String key = "reservation:" + reservationId + ":" + chargerId;

        // when
        reservationCacheRepository.createReservationGracePeriod(reservationId, chargerId, ReservationConst.gracePeriod);

        // Then
        String value = redisTemplate.opsForValue().get(key);
        assertThat(value).isEqualTo("true");
    }

    @DisplayName("모든 예약을 삭제한다.")
    @Test
    public void deleteAllReservations() {
        // given
        Long reservationId = 1L;
        Long chargerId = 1L;
        String key = "reservation:" + reservationId + ":" + chargerId;
        reservationCacheRepository.createReservationGracePeriod(reservationId, chargerId, ReservationConst.gracePeriod);

        // when
        reservationCacheRepository.deleteAllReservations();

        // Then
        String value = redisTemplate.opsForValue().get(key);
        assertThat(value).isNull();
    }

}

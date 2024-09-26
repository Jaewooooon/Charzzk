package com.ssafy.charzzk.api.controller.reservation;

import com.ssafy.charzzk.ControllerTestSupport;
import com.ssafy.charzzk.api.service.response.ReservationCheckTimeResponse;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.springframework.http.MediaType;
import org.springframework.security.test.context.support.WithMockUser;
import org.springframework.test.web.servlet.ResultActions;

import java.time.LocalDateTime;

import static org.mockito.ArgumentMatchers.*;
import static org.mockito.BDDMockito.given;
import static org.springframework.test.web.servlet.request.MockMvcRequestBuilders.get;
import static org.springframework.test.web.servlet.result.MockMvcResultHandlers.print;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.jsonPath;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.status;

class ReservationControllerTest extends ControllerTestSupport {

    @DisplayName("예약 가능시간을 성공적으로 조회한다.")
    @WithMockUser
    @Test
    void checkTime() throws Exception {
        // given
        ReservationCheckTimeResponse response = ReservationCheckTimeResponse.builder()
                .startTime(LocalDateTime.of(2021, 8, 1, 0, 0, 0))
                .endTime(LocalDateTime.of(2021, 8, 1, 2, 0, 0))
                .isPossibleNow(true)
                .build();

        given(reservationService.checkTime(anyLong(), anyLong(), anyBoolean(), anyInt(), any())).willReturn(response);

        // when
        ResultActions performs = mockMvc.perform(get("/api/v1/reservations/check-time")
                .param("parkingLotId", "1")
                .param("carId", "1")
                .param("fullCharge", "true")
                .param("time", "0")
                .contentType(MediaType.APPLICATION_JSON));

        // then
        performs.andDo(print())
                .andExpect(status().isOk())
                .andExpect(jsonPath("$.code").value("200"))
                .andExpect(jsonPath("$.status").value("OK"))
                .andExpect(jsonPath("$.message").value("OK"))
                .andExpect(jsonPath("$.data").isNotEmpty());

    }

}
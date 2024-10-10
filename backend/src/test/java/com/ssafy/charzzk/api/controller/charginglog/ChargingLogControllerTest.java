package com.ssafy.charzzk.api.controller.charginglog;

import com.ssafy.charzzk.ControllerTestSupport;
import com.ssafy.charzzk.api.service.car.response.CarChargingLogResponse;
import com.ssafy.charzzk.api.service.car.response.CarTypeChargingLogResponse;
import com.ssafy.charzzk.api.service.charginglog.response.ChargingLogResponse;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.springframework.http.MediaType;
import org.springframework.security.test.context.support.WithMockUser;
import org.springframework.test.web.servlet.ResultActions;

import java.time.LocalDateTime;
import java.util.List;

import static org.mockito.ArgumentMatchers.any;
import static org.mockito.BDDMockito.given;
import static org.springframework.restdocs.mockmvc.RestDocumentationRequestBuilders.get;
import static org.springframework.security.test.web.servlet.request.SecurityMockMvcRequestPostProcessors.csrf;
import static org.springframework.test.web.servlet.result.MockMvcResultHandlers.print;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.jsonPath;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.status;

class ChargingLogControllerTest extends ControllerTestSupport {

    @DisplayName("충전 로그 리스트를 성공적으로 조회한다.")
    @WithMockUser
    @Test
    void getChargingLogList() throws Exception {

        // given
        CarTypeChargingLogResponse carType = CarTypeChargingLogResponse.builder()
                .id(1L)
                .name("아이오닉")
                .build();

        CarChargingLogResponse car = CarChargingLogResponse.builder()
                .id(1L)
                .carType(carType)
                .number("1234")
                .nickname("닉네임")
                .build();

        ChargingLogResponse chargingLogResponse1 = ChargingLogResponse.builder()
                .id(1L)
                .car(car)
                .startTime(LocalDateTime.now())
                .endTime(LocalDateTime.now().plusHours(1))
                .chargeAmount(50.0)
                .chargeCost(15000L)
                .build();

        ChargingLogResponse chargingLogResponse2 = ChargingLogResponse.builder()
                .id(2L)
                .car(car)
                .startTime(LocalDateTime.now().minusDays(1))
                .endTime(LocalDateTime.now().minusDays(1).plusHours(2))
                .chargeAmount(100.0)
                .chargeCost(30000L)
                .build();

        List<ChargingLogResponse> chargingLogResponseList = List.of(chargingLogResponse1, chargingLogResponse2);

        given(chargingLogService.getChargingLogList(any())).willReturn(chargingLogResponseList);

        // when
        ResultActions perform = mockMvc.perform(
                get("/api/v1/charging-log")
                        .with(csrf())
                        .contentType(MediaType.APPLICATION_JSON)
        );

        // then
        perform.andDo(print())
                .andExpect(status().isOk())
                .andExpect(jsonPath("$.code").value("200"))
                .andExpect(jsonPath("$.status").value("OK"))
                .andExpect(jsonPath("$.message").value("OK"))
                .andExpect(jsonPath("$.data").isArray())
                .andExpect(jsonPath("$.data[0].id").value(1L))
                .andExpect(jsonPath("$.data[0].car.number").value("1234"))
                .andExpect(jsonPath("$.data[0].chargeAmount").value(50.0))
                .andExpect(jsonPath("$.data[0].chargeCost").value(15000L))
                .andExpect(jsonPath("$.data[1].id").value(2L))
                .andExpect(jsonPath("$.data[1].car.number").value("1234"))
                .andExpect(jsonPath("$.data[1].chargeAmount").value(100.0))
                .andExpect(jsonPath("$.data[1].chargeCost").value(30000L));
    }
}
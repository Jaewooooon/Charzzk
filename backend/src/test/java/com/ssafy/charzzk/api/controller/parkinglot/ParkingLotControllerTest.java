package com.ssafy.charzzk.api.controller.parkinglot;

import com.ssafy.charzzk.ControllerTestSupport;
import com.ssafy.charzzk.api.controller.parkinglot.request.ParkingLotListRequest;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.springframework.http.MediaType;
import org.springframework.security.test.context.support.WithMockUser;
import org.springframework.test.web.servlet.ResultActions;

import static org.springframework.security.test.web.servlet.request.SecurityMockMvcRequestPostProcessors.csrf;
import static org.springframework.test.web.servlet.request.MockMvcRequestBuilders.get;
import static org.springframework.test.web.servlet.result.MockMvcResultHandlers.print;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.jsonPath;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.status;


class ParkingLotControllerTest extends ControllerTestSupport {

    @DisplayName("주차장 목록을 조회에 성공한다")
    @WithMockUser
    @Test
    void getParkingLotList() throws Exception {
        // given
        ParkingLotListRequest request = ParkingLotListRequest.builder()
                .latitude(37.123456)
                .longitude(127.123456)
                .build();

        // when
        ResultActions perform = mockMvc.perform(
                get("/api/v1/parking-lot")
                        .with(csrf())
                        .content(objectMapper.writeValueAsString(request))
                        .contentType(MediaType.APPLICATION_JSON)
        );

        // then
        perform.andDo(print())
                .andExpect(status().isOk())
                .andExpect(jsonPath("$.code").value("200"))
                .andExpect(jsonPath("$.status").value("OK"))
                .andExpect(jsonPath("$.message").value("OK"))
                .andExpect(jsonPath("$.data").isArray());
    }

    @DisplayName("주차장 목록을 조회할 때 위도는 필수값이다")
    @WithMockUser
    @Test
    void getParkingLotListWithoutLatitude() throws Exception {
        // given
        ParkingLotListRequest request = ParkingLotListRequest.builder()
                .longitude(127.123456)
                .build();

        // when
        ResultActions perform = mockMvc.perform(
                get("/api/v1/parking-lot")
                        .with(csrf())
                        .content(objectMapper.writeValueAsString(request))
                        .contentType(MediaType.APPLICATION_JSON)
        );

        // then
        perform.andDo(print())
                .andExpect(status().isBadRequest())
                .andExpect(jsonPath("$.code").value("400"))
                .andExpect(jsonPath("$.status").value("BAD_REQUEST"))
                .andExpect(jsonPath("$.message").value("위도는 필수입니다."))
                .andExpect(jsonPath("$.data").isEmpty());
    }

    @DisplayName("주차장 목록을 조회할 때 경도는 필수값이다")
    @WithMockUser
    @Test
    void getParkingLotListWithoutLongitude() throws Exception {
        // given
        ParkingLotListRequest request = ParkingLotListRequest.builder()
                .latitude(37.123456)
                .build();

        // when
        ResultActions perform = mockMvc.perform(
                get("/api/v1/parking-lot")
                        .with(csrf())
                        .content(objectMapper.writeValueAsString(request))
                        .contentType(MediaType.APPLICATION_JSON)
        );

        // then
        perform.andDo(print())
                .andExpect(status().isBadRequest())
                .andExpect(jsonPath("$.code").value("400"))
                .andExpect(jsonPath("$.status").value("BAD_REQUEST"))
                .andExpect(jsonPath("$.message").value("경도는 필수입니다."))
                .andExpect(jsonPath("$.data").isEmpty());
    }



}
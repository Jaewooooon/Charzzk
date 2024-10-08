package com.ssafy.charzzk.api.controller.parkinglot;

import com.ssafy.charzzk.ControllerTestSupport;
import com.ssafy.charzzk.api.controller.parkinglot.request.ParkingLotListRequest;
import com.ssafy.charzzk.api.service.charger.response.ChargerResponse;
import com.ssafy.charzzk.api.service.parkinglot.response.ParkingLotListResponse;
import com.ssafy.charzzk.api.service.parkinglot.response.ParkingLotResponse;
import com.ssafy.charzzk.api.service.parkinglot.response.ParkingSpotListResponse;
import com.ssafy.charzzk.domain.charger.ChargerStatus;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.springframework.http.MediaType;
import org.springframework.security.test.context.support.WithMockUser;
import org.springframework.test.web.servlet.ResultActions;

import java.util.List;

import static org.mockito.ArgumentMatchers.any;
import static org.mockito.BDDMockito.given;
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
        // when
        ResultActions perform = mockMvc.perform(
                get("/api/v1/parking-lot")
                        .with(csrf())
                        .param("latitude", "37.123456")
                        .param("longitude", "127.123456")
        );

        // then
        perform.andDo(print())
                .andExpect(status().isOk())
                .andExpect(jsonPath("$.code").value("200"))
                .andExpect(jsonPath("$.status").value("OK"))
                .andExpect(jsonPath("$.message").value("OK"))
                .andExpect(jsonPath("$.data").isArray());
    }

    @DisplayName("주차장 검색에 성공한다")
    @WithMockUser
    @Test
    void getParkingLotListWithKeyword() throws Exception {
        // given
        ParkingLotListRequest request = ParkingLotListRequest.builder()
                .latitude(37.123456)
                .longitude(127.123456)
                .build();

        List<ParkingLotListResponse> response = List.of(
                ParkingLotListResponse.builder()
                        .id(1L)
                        .name("주차장1")
                        .build(),
                ParkingLotListResponse.builder()
                        .id(2L)
                        .name("주차장2")
                        .build()
        );

        given(parkingLotService.getParkingLotList(any(), any(), any())).willReturn(response);

        // when
        ResultActions perform = mockMvc.perform(
                get("/api/v1/parking-lot")
                        .with(csrf())
                        .param("keyword", "주차장")
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


    @DisplayName("주차장 상세조회에 성공한다")
    @WithMockUser
    @Test
    void getParkingLot() throws Exception {
        // given
        List<ParkingSpotListResponse> parkingSpotListResponseList = List.of(
                ParkingSpotListResponse.builder()
                        .id(1L)
                        .name("A11")
                        .build(),
                ParkingSpotListResponse.builder()
                        .id(2L)
                        .name("A12")
                        .build()
        );

        ParkingLotResponse response = ParkingLotResponse.builder()
                .id(1L)
                .name("주차장1")
                .parkingMapImage("주차장1 이미지")
                .parkingSpots(parkingSpotListResponseList)
                .build();

        given(parkingLotService.getParkingLot(any())).willReturn(response);


        // when
        ResultActions perform = mockMvc.perform(
                get("/api/v1/parking-lot/{parkingLotId}", 1L)
                        .contentType(MediaType.APPLICATION_JSON)
        );

        // then
        perform.andDo(print())
                .andExpect(status().isOk())
                .andExpect(jsonPath("$.code").value("200"))
                .andExpect(jsonPath("$.status").value("OK"))
                .andExpect(jsonPath("$.message").value("OK"))
                .andExpect(jsonPath("$.data.id").value(response.getId()))
                .andExpect(jsonPath("$.data.name").value(response.getName()))
                .andExpect(jsonPath("$.data.parkingMapImage").value(response.getParkingMapImage()))
                .andExpect(jsonPath("$.data.parkingSpots").isArray());
    }

    @DisplayName("주차장 ID로 조회하면 해당 주차장에 속한 충전 로봇 목록이 정상적으로 반환된다.")
    @WithMockUser
    @Test
    void getChargerList() throws Exception {
        // given
        ChargerResponse chargerResponse1 = ChargerResponse.builder()
                .id(1L)
                .serialNumber("1234A")
                .battery(80)
                .status(ChargerStatus.WAITING)
                .build();

        ChargerResponse chargerResponse2 = ChargerResponse.builder()
                .id(2L)
                .serialNumber("1234B")
                .battery(90)
                .status(ChargerStatus.CHARGER_CHARGING)
                .build();

        List<ChargerResponse> chargerList = List.of(chargerResponse1, chargerResponse2);

        given(parkingLotService.getChargerList(any(Long.class))).willReturn(chargerList);

        // when
        ResultActions perform = mockMvc.perform(
                get("/api/v1/parking-lot/{parkingLotId}/chargers", 1L)
                        .with(csrf())
                        .contentType(MediaType.APPLICATION_JSON)
        );

        // then
        perform.andDo(print())
                .andExpect(status().isOk())
                .andExpect(jsonPath("$.code").value("200"))
                .andExpect(jsonPath("$.status").value("OK"))
                .andExpect(jsonPath("$.message").value("OK"))
                .andExpect(jsonPath("$.data").exists());
    }
}
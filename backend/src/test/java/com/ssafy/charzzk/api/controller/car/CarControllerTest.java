package com.ssafy.charzzk.api.controller.car;

import com.ssafy.charzzk.ControllerTestSupport;
import com.ssafy.charzzk.api.controller.car.request.CarRequest;
import com.ssafy.charzzk.api.service.car.response.CarResponse;
import com.ssafy.charzzk.api.service.car.response.CarTypeResponse;
import com.ssafy.charzzk.domain.car.CarType;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.springframework.http.MediaType;
import org.springframework.security.test.context.support.WithMockUser;
import org.springframework.test.web.servlet.ResultActions;

import java.util.Arrays;
import java.util.List;

import static org.mockito.ArgumentMatchers.any;
import static org.mockito.BDDMockito.given;
import static org.springframework.security.test.web.servlet.request.SecurityMockMvcRequestPostProcessors.csrf;
import static org.springframework.test.web.servlet.request.MockMvcRequestBuilders.*;
import static org.springframework.test.web.servlet.result.MockMvcResultHandlers.print;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.jsonPath;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.status;

class CarControllerTest extends ControllerTestSupport {

    @DisplayName("올바른 정보로 차량을 등록하면 등록에 성공한다.")
    @WithMockUser
    @Test
    public void createCar() throws Exception {
        // given
        CarRequest request = CarRequest.builder()
                .carTypeId(1L)
                .number("11다1111")
                .nickname("콩이")
                .build();

        CarType carType = CarType.builder()
                .image("example image")
                .name("테슬라 모델 3")
                .build();

        CarResponse response = CarResponse.builder()
                .id(1L)
                .carType(carType)
                .number("11다1111")
                .nickname("콩이")
                .build();

        given(carService.getCar(any())).willReturn(response);

        // when
        ResultActions perform = mockMvc.perform(
                post("/api/v1/cars")
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
                .andExpect(jsonPath("$.data").exists());
    }

    @DisplayName("차량을 등록할 때 차량 기종 ID는 필수값이다.")
    @WithMockUser
    @Test
    public void createCarWithoutCarTypeId() throws Exception {
        // given
        CarRequest request = CarRequest.builder()
                .number("11다1111")
                .nickname("콩이")
                .build();

        // when
        ResultActions perform = mockMvc.perform(
                post("/api/v1/cars")
                        .with(csrf())
                        .content(objectMapper.writeValueAsString(request))
                        .contentType(MediaType.APPLICATION_JSON)
        );

        // then
        perform.andDo(print())
                .andExpect(status().isBadRequest())
                .andExpect(jsonPath("$.code").value("400"))
                .andExpect(jsonPath("$.status").value("BAD_REQUEST"))
                .andExpect(jsonPath("$.message").value("차량 기종은 필수입니다."))
                .andExpect(jsonPath("$.data").isEmpty());
    }

    @DisplayName("차량을 등록할 때 차량 번호는 필수값이다.")
    @WithMockUser
    @Test
    public void createCarWithoutNumber() throws Exception {
        // given
        CarRequest request = CarRequest.builder()
                .carTypeId(1L)
                .nickname("콩이")
                .build();

        // when
        ResultActions perform = mockMvc.perform(
                post("/api/v1/cars")
                        .with(csrf())
                        .content(objectMapper.writeValueAsString(request))
                        .contentType(MediaType.APPLICATION_JSON)
        );

        // then
        perform.andDo(print())
                .andExpect(status().isBadRequest())
                .andExpect(jsonPath("$.code").value("400"))
                .andExpect(jsonPath("$.status").value("BAD_REQUEST"))
                .andExpect(jsonPath("$.message").value("차량 번호는 필수입니다."))
                .andExpect(jsonPath("$.data").isEmpty());
    }

    @DisplayName("차량을 등록할 때 차량 번호가 빈 문자열이면 예외가 발생한다.")
    @WithMockUser
    @Test
    public void createCarWithBlankNumber() throws Exception {
        // given
        CarRequest request = CarRequest.builder()
                .carTypeId(1L)
                .number("")
                .nickname("콩이")
                .build();

        // when
        ResultActions perform = mockMvc.perform(
                post("/api/v1/cars")
                        .with(csrf())
                        .content(objectMapper.writeValueAsString(request))
                        .contentType(MediaType.APPLICATION_JSON)
        );

        // then
        perform.andDo(print())
                .andExpect(status().isBadRequest())
                .andExpect(jsonPath("$.code").value("400"))
                .andExpect(jsonPath("$.status").value("BAD_REQUEST"))
                .andExpect(jsonPath("$.message").value("차량 번호는 필수입니다."))
                .andExpect(jsonPath("$.data").isEmpty());
    }

    @DisplayName("차종을 이름으로 검색하면 해당 차종 목록을 반환한다.")
    @WithMockUser
    @Test
    public void getCarTypesByName() throws Exception {
        // given
        CarTypeResponse carType1 = CarTypeResponse.builder()
                .id(1L)
                .name("테슬라 모델 3")
                .image("cars/image1")
                .build();

        CarTypeResponse carType2 = CarTypeResponse.builder()
                .id(2L)
                .name("테슬라 모델 Y")
                .image("cars/image2")
                .build();

        List<CarTypeResponse> carTypeResponses = Arrays.asList(carType1,carType2);

        given(carService.getCarTypes(any())).willReturn(carTypeResponses);

        // when
        ResultActions perform = mockMvc.perform(
                get("/api/v1/car-types")
                        .param("q", "테슬라")
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
                .andExpect(jsonPath("$.data[0].name").value("테슬라 모델 3"))
                .andExpect(jsonPath("$.data[1].id").value(2L))
                .andExpect(jsonPath("$.data[1].name").value("테슬라 모델 Y"));
    }
}
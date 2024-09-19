package com.ssafy.charzzk.api.controller.car;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.ssafy.charzzk.ControllerTestSupport;
import com.ssafy.charzzk.api.controller.car.request.CarRequest;
import com.ssafy.charzzk.api.service.car.request.CarServiceRequest;
import com.ssafy.charzzk.api.service.car.response.CarResponse;
import com.ssafy.charzzk.domain.car.Car;
import com.ssafy.charzzk.domain.car.CarType;
import com.ssafy.charzzk.domain.user.User;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.springframework.http.MediaType;
import org.springframework.security.test.context.support.WithMockUser;
import org.springframework.test.web.servlet.ResultActions;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.BDDMockito.given;
import static org.springframework.security.test.web.servlet.request.SecurityMockMvcRequestPostProcessors.csrf;
import static org.springframework.test.web.servlet.request.MockMvcRequestBuilders.patch;
import static org.springframework.test.web.servlet.request.MockMvcRequestBuilders.post;
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
//        perform.andDo(print())
//                .andExpect(status().isOk())
//                .andExpect(jsonPath("$.code").value("200"))
//                .andExpect(jsonPath("$.status").value("OK"))
//                .andExpect(jsonPath("$.message").value("OK"));
    }

}
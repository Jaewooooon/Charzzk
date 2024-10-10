package com.ssafy.charzzk.api.controller.car;

import com.ssafy.charzzk.ControllerTestSupport;
import com.ssafy.charzzk.api.controller.car.request.CarRequest;
import com.ssafy.charzzk.api.service.car.response.CarListResponse;
import com.ssafy.charzzk.api.service.car.response.CarReservationStatusResponse;
import com.ssafy.charzzk.api.service.car.response.CarResponse;
import com.ssafy.charzzk.api.service.car.response.CarTypeResponse;
import com.ssafy.charzzk.domain.car.CarType;
import com.ssafy.charzzk.domain.reservation.ReservationStatus;
import com.ssafy.charzzk.domain.user.User;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.springframework.http.MediaType;
import org.springframework.security.test.context.support.WithMockUser;
import org.springframework.test.web.servlet.ResultActions;

import java.time.LocalDateTime;
import java.util.Arrays;
import java.util.List;

import static org.mockito.ArgumentMatchers.any;
import static org.mockito.BDDMockito.given;
import static org.mockito.BDDMockito.willDoNothing;
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

        List<CarTypeResponse> carTypeResponses = Arrays.asList(carType1, carType2);

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

    @DisplayName("차량을 수정하면 정상적으로 수정된다.")
    @WithMockUser
    @Test
    public void updateCar() throws Exception {
        // given
        CarRequest request = CarRequest.builder()
                .carTypeId(1L)
                .number("11가1111")
                .nickname("새로운 콩이")
                .build();

        CarType carType = CarType.builder()
                .image("example image")
                .name("테슬라 모델 3")
                .build();

        CarResponse response = CarResponse.builder()
                .id(1L)
                .carType(carType)
                .number("11가1111")
                .nickname("새로운 콩이")
                .build();

        given(carService.getCar(any())).willReturn(response);

        // when
        ResultActions perform = mockMvc.perform(
                patch("/api/v1/cars/{carId}", 1L)
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
                .andExpect(jsonPath("$.data.number").value("11가1111"));
    }

    @DisplayName("차량을 수정할 때 차량 기종 ID는 필수값이다.")
    @WithMockUser
    @Test
    public void updateCarWithoutCarTypeId() throws Exception {
        // given
        CarRequest request = CarRequest.builder()
                .number("11다1111")
                .nickname("콩이")
                .build();

        // when
        ResultActions perform = mockMvc.perform(
                patch("/api/v1/cars/{carId}", 1L)
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

    @DisplayName("차량을 수정할 때 차량 번호는 필수값이다.")
    @WithMockUser
    @Test
    public void updateCarWithoutNumber() throws Exception {
        // given
        CarRequest request = CarRequest.builder()
                .carTypeId(1L)
                .nickname("콩이")
                .build();

        // when
        ResultActions perform = mockMvc.perform(
                patch("/api/v1/cars/{carId}", 1L)
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

    @DisplayName("차량을 수정할 때 차량 번호가 빈 문자열이면 예외가 발생한다.")
    @WithMockUser
    @Test
    public void updateCarWithBlankNumber() throws Exception {
        // given
        CarRequest request = CarRequest.builder()
                .carTypeId(1L)
                .number("")
                .nickname("콩이")
                .build();

        // when
        ResultActions perform = mockMvc.perform(
                patch("/api/v1/cars/{carId}", 1L)
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

    @DisplayName("차량을 삭제하면 정상적으로 삭제된다.")
    @WithMockUser
    @Test
    public void deleteCar() throws Exception {
        // given
        Long carId = 1L;

        // 차량 삭제 메서드 호출해도 아무 일 없도록 함
        willDoNothing().given(carService).deleteCar(any(Long.class), any(User.class));

        // when
        ResultActions perform = mockMvc.perform(
                delete("/api/v1/cars/{carId}", carId)
                        .with(csrf())
                        .contentType(MediaType.APPLICATION_JSON)
        );

        // then
        perform.andDo(print())
                .andExpect(status().isOk())
                .andExpect(jsonPath("$.code").value("200"))
                .andExpect(jsonPath("$.status").value("OK"))
                .andExpect(jsonPath("$.message").value("OK"))
                .andExpect(jsonPath("$.data").isEmpty());
    }

    @DisplayName("사용자의 차량 목록을 조회하면 정상적으로 반환된다.")
    @WithMockUser
    @Test
    public void getCarList() throws Exception {
        // given
        CarTypeResponse carTypeResponse = CarTypeResponse.builder()
                .id(1L)
                .name("테슬라 모델 3")
                .image("cars/image1")
                .build();

        CarListResponse carResponse1 = CarListResponse.builder()
                .id(1L)
                .carType(carTypeResponse)
                .number("11다1111")
                .nickname("콩이")
                .isCharging(false)
                .chargeCost(10000L)
                .chargeAmount(500.0)
                .build();

        CarListResponse carResponse2 = CarListResponse.builder()
                .id(2L)
                .carType(carTypeResponse)
                .number("22나2222")
                .nickname("둥이")
                .isCharging(true)
                .chargeCost(20000L)
                .chargeAmount(1000.0)
                .build();

        List<CarListResponse> carList = List.of(carResponse1, carResponse2);

        LocalDateTime startOfMonth = LocalDateTime.of(2024, 9, 1, 0, 0, 0);
        LocalDateTime endOfMonth = LocalDateTime.of(2024, 9, 30, 23, 59, 59);

        given(carService.getCarList(any(User.class), any(LocalDateTime.class), any(LocalDateTime.class))).willReturn(carList);

        // when
        ResultActions perform = mockMvc.perform(
                get("/api/v1/cars/me")
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
                .andExpect(jsonPath("$.data[0].number").value("11다1111"))
                .andExpect(jsonPath("$.data[0].nickname").value("콩이"))
                .andExpect(jsonPath("$.data[0].charging").value(false))
                .andExpect(jsonPath("$.data[0].chargeCost").value(10000L))
                .andExpect(jsonPath("$.data[0].chargeAmount").value(500L))
                .andExpect(jsonPath("$.data[1].id").value(2L))
                .andExpect(jsonPath("$.data[1].number").value("22나2222"))
                .andExpect(jsonPath("$.data[1].nickname").value("둥이"))
                .andExpect(jsonPath("$.data[1].charging").value(true))
                .andExpect(jsonPath("$.data[1].chargeCost").value(20000L))
                .andExpect(jsonPath("$.data[1].chargeAmount").value(1000L));
    }


    @DisplayName("현재 자동차의 충전 상태를 반환한다.")
    @WithMockUser
    @Test
    public void getCarChargingStatus() throws Exception {
        // given
        CarReservationStatusResponse response = CarReservationStatusResponse.builder()
                .battery(30)
                .startTime(LocalDateTime.of(2024, 9, 1, 0, 0, 0))
                .endTime(LocalDateTime.of(2024, 9, 1, 1, 0, 0))
                .status(ReservationStatus.CHARGING.name())
                .build();

        given(carService.getCarChargingStatus(any(User.class), any(Long.class))).willReturn(response);

        // when
        ResultActions perform = mockMvc.perform(
                get("/api/v1/cars/{carId}", 1)
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
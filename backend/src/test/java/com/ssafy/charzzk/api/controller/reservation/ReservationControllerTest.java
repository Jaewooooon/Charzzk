package com.ssafy.charzzk.api.controller.reservation;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.ssafy.charzzk.ControllerTestSupport;
import com.ssafy.charzzk.api.controller.reservation.request.ReservationConfirmRequest;
import com.ssafy.charzzk.api.controller.reservation.request.ReservationRequest;
import com.ssafy.charzzk.api.service.car.response.CarResponse;
import com.ssafy.charzzk.api.service.reservation.response.ReservationResponse;
import com.ssafy.charzzk.domain.car.Car;
import com.ssafy.charzzk.domain.car.CarType;
import com.ssafy.charzzk.domain.reservation.Reservation;
import com.ssafy.charzzk.domain.reservation.ReservationStatus;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.springframework.http.MediaType;
import org.springframework.security.test.context.support.WithMockUser;
import org.springframework.test.web.servlet.ResultActions;

import java.time.LocalDateTime;

import static org.mockito.ArgumentMatchers.any;
import static org.mockito.BDDMockito.given;
import static org.springframework.security.test.web.servlet.request.SecurityMockMvcRequestPostProcessors.csrf;
import static org.springframework.test.web.servlet.request.MockMvcRequestBuilders.patch;
import static org.springframework.test.web.servlet.request.MockMvcRequestBuilders.post;
import static org.springframework.test.web.servlet.result.MockMvcResultHandlers.print;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.jsonPath;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.status;

class ReservationControllerTest extends ControllerTestSupport {

    @DisplayName("예약을 성공적으로 만든다.")
    @WithMockUser
    @Test
    void createReservation() throws Exception {

        // given
        ReservationRequest request = ReservationRequest.builder()
                .parkingSpotId(1L)
                .carId(1L)
                .parkingLotId(1L)
                .fullCharge(true)
                .time(0)
                .build();

        CarType carType = CarType.builder()
                .id(1L)
                .name("아이오닉")
                .image("image")
                .build();

        Car car = Car.builder()
                .carType(carType)
                .number("1234")
                .nickname("nickname")
                .build();

        ReservationResponse response = ReservationResponse.builder()
                .id(1L)
                .car(CarResponse.from(car))
                .startTime(LocalDateTime.of(2021, 1, 1, 0, 0))
                .endTime(LocalDateTime.of(2021, 1, 1, 2, 0))
                .status(ReservationStatus.PENDING.name())
                .build();

        given(reservationService.create(any(), any(), any())).willReturn(Reservation.builder().id(1L).build());
        given(reservationService.getReservation(any())).willReturn(response);

        // when
        ResultActions perform = mockMvc.perform(
                post("/api/v1/reservations")
                        .with(csrf())
                        .content(objectMapper.writeValueAsString(request))
                        .contentType(MediaType.APPLICATION_JSON));

        // then
        perform.andDo(print())
                .andExpect(status().isOk())
                .andExpect(jsonPath("$.code").value("200"))
                .andExpect(jsonPath("$.status").value("OK"))
                .andExpect(jsonPath("$.message").value("OK"))
                .andExpect(jsonPath("$.data").isNotEmpty());

    }

    @DisplayName("예약을 만들 때 자동차 아이디는 필수값이다.")
    @WithMockUser
    @Test
    void createReservationWithoutCarId() throws Exception {

        // given
        ReservationRequest request = ReservationRequest.builder()
                .parkingSpotId(1L)
                .parkingLotId(1L)
                .fullCharge(true)
                .time(0)
                .build();

        CarType carType = CarType.builder()
                .id(1L)
                .name("아이오닉")
                .image("image")
                .build();

        Car car = Car.builder()
                .carType(carType)
                .number("1234")
                .nickname("nickname")
                .build();

        ReservationResponse response = ReservationResponse.builder()
                .id(1L)
                .car(CarResponse.from(car))
                .startTime(LocalDateTime.of(2021, 1, 1, 0, 0))
                .endTime(LocalDateTime.of(2021, 1, 1, 2, 0))
                .status(ReservationStatus.PENDING.name())
                .build();

        given(reservationService.getReservation(any())).willReturn(response);

        // when
        ResultActions perform = mockMvc.perform(
                post("/api/v1/reservations")
                        .with(csrf())
                        .content(objectMapper.writeValueAsString(request))
                        .contentType(MediaType.APPLICATION_JSON));

        // then
        perform.andDo(print())
                .andExpect(status().isBadRequest())
                .andExpect(jsonPath("$.code").value("400"))
                .andExpect(jsonPath("$.status").value("BAD_REQUEST"))
                .andExpect(jsonPath("$.message").value("차량 ID는 필수 값입니다."))
                .andExpect(jsonPath("$.data").isEmpty());
    }

    @DisplayName("예약을 만들 때 주차장 아이디는 필수값이다.")
    @WithMockUser
    @Test
    void createReservationWithoutParkingLotId() throws Exception {

        // given
        ReservationRequest request = ReservationRequest.builder()
                .parkingSpotId(1L)
                .carId(1L)
                .fullCharge(true)
                .time(0)
                .build();

        CarType carType = CarType.builder()
                .id(1L)
                .name("아이오닉")
                .image("image")
                .build();

        Car car = Car.builder()
                .carType(carType)
                .number("1234")
                .nickname("nickname")
                .build();

        ReservationResponse response = ReservationResponse.builder()
                .id(1L)
                .car(CarResponse.from(car))
                .startTime(LocalDateTime.of(2021, 1, 1, 0, 0))
                .endTime(LocalDateTime.of(2021, 1, 1, 2, 0))
                .status(ReservationStatus.PENDING.name())
                .build();

        given(reservationService.getReservation(any())).willReturn(response);

        // when
        ResultActions perform = mockMvc.perform(
                post("/api/v1/reservations")
                        .with(csrf())
                        .content(objectMapper.writeValueAsString(request))
                        .contentType(MediaType.APPLICATION_JSON));

        // then
        perform.andDo(print())
                .andExpect(status().isBadRequest())
                .andExpect(jsonPath("$.code").value("400"))
                .andExpect(jsonPath("$.status").value("BAD_REQUEST"))
                .andExpect(jsonPath("$.message").value("주차장 ID는 필수 값입니다."))
                .andExpect(jsonPath("$.data").isEmpty());
    }

    @DisplayName("예약 아이디로 예약을 확정한다.")
    @WithMockUser
    @Test
    public void confirmReservation() throws Exception {

        // given
        ReservationConfirmRequest request = ReservationConfirmRequest.builder()
                .reservationId(1L)
                .build();

        ReservationResponse response = ReservationResponse.builder()
                .id(1L)
                .car(CarResponse.builder()
                        .id(1L)
                        .carType(CarType.builder()
                                .id(1L)
                                .name("아이오닉")
                                .image("image")
                                .build())
                        .number("1234")
                        .nickname("nickname")
                        .build())
                .startTime(LocalDateTime.of(2021, 1, 1, 0, 0))
                .endTime(LocalDateTime.of(2021, 1, 1, 2, 0))
                .status(ReservationStatus.WAITING.name())
                .build();

        given(reservationService.confirm(any(), any())).willReturn(Reservation.builder().id(1L).build());
        given(reservationService.getReservation(any())).willReturn(response);

        // when
        ResultActions perform = mockMvc.perform(
                patch("/api/v1/reservations/1")
                        .with(csrf())
                        .content(objectMapper.writeValueAsString(request))
                        .contentType(MediaType.APPLICATION_JSON));

        // then
        perform.andDo(print())
                .andExpect(status().isOk())
                .andExpect(jsonPath("$.code").value("200"))
                .andExpect(jsonPath("$.status").value("OK"))
                .andExpect(jsonPath("$.message").value("OK"))
                .andExpect(jsonPath("$.data").isNotEmpty());
    }
}

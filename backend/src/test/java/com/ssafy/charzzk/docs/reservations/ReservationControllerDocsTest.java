package com.ssafy.charzzk.docs.reservations;

import com.epages.restdocs.apispec.ResourceSnippetParameters;
import com.ssafy.charzzk.api.controller.reservation.ReservationController;
import com.ssafy.charzzk.api.controller.reservation.request.ReservationConfirmRequest;
import com.ssafy.charzzk.api.controller.reservation.request.ReservationRequest;
import com.ssafy.charzzk.api.service.car.response.CarResponse;
import com.ssafy.charzzk.api.service.reservation.ReservationService;
import com.ssafy.charzzk.api.service.reservation.response.ReservationResponse;
import com.ssafy.charzzk.docs.RestDocsSupport;
import com.ssafy.charzzk.domain.car.Car;
import com.ssafy.charzzk.domain.car.CarType;
import com.ssafy.charzzk.domain.reservation.Reservation;
import com.ssafy.charzzk.domain.reservation.ReservationStatus;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.springframework.http.MediaType;
import org.springframework.restdocs.payload.JsonFieldType;
import org.springframework.security.test.context.support.WithMockUser;
import org.springframework.test.web.servlet.ResultActions;

import java.time.LocalDateTime;

import static com.epages.restdocs.apispec.MockMvcRestDocumentationWrapper.document;
import static com.epages.restdocs.apispec.ResourceDocumentation.resource;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.BDDMockito.given;
import static org.mockito.Mockito.mock;
import static org.springframework.restdocs.headers.HeaderDocumentation.headerWithName;
import static org.springframework.restdocs.mockmvc.RestDocumentationRequestBuilders.patch;
import static org.springframework.restdocs.mockmvc.RestDocumentationRequestBuilders.post;
import static org.springframework.restdocs.operation.preprocess.Preprocessors.preprocessResponse;
import static org.springframework.restdocs.operation.preprocess.Preprocessors.prettyPrint;
import static org.springframework.restdocs.payload.PayloadDocumentation.fieldWithPath;
import static org.springframework.security.test.web.servlet.request.SecurityMockMvcRequestPostProcessors.csrf;
import static org.springframework.test.web.servlet.result.MockMvcResultHandlers.print;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.status;

public class ReservationControllerDocsTest extends RestDocsSupport {

    private final ReservationService reservationService = mock(ReservationService.class);

    @Override
    protected Object initController() {
        return new ReservationController(reservationService);
    }

    @DisplayName("예약을 성공적으로 만든다.")
    @Test
    void createReservation() throws Exception {

        // given
        ReservationRequest request = ReservationRequest.builder()
                .carId(1L)
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
                .id(1L)
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
                        .header("Authorization", "Bearer token")
                        .content(objectMapper.writeValueAsString(request))
                        .contentType(MediaType.APPLICATION_JSON));

        // then
        perform
                .andDo(print())
                .andExpect(status().isOk())
                .andDo(document("reservation-create",
                        preprocessResponse(prettyPrint()),
                        resource(ResourceSnippetParameters.builder()
                                .tag("Reservation")
                                .summary("예약 생성")
                                .requestHeaders(
                                        headerWithName("Authorization").description("JWT 토큰 (Bearer)")
                                )
                                .requestFields(
                                        fieldWithPath("parkingSpotId").type(JsonFieldType.NUMBER)
                                                .description("주차칸 아이디"),
                                        fieldWithPath("carId").type(JsonFieldType.NUMBER)
                                                .description("자동차 아이디"),
                                        fieldWithPath("parkingLotId").type(JsonFieldType.NUMBER)
                                                .description("주차장 아이디"),
                                        fieldWithPath("fullCharge").type(JsonFieldType.BOOLEAN)
                                                .description("풀충전 여부"),
                                        fieldWithPath("time").type(JsonFieldType.NUMBER)
                                                .optional().description("충전 시간")
                                )
                                .responseFields(
                                        fieldWithPath("code").type(JsonFieldType.NUMBER)
                                                .description("코드"),
                                        fieldWithPath("status").type(JsonFieldType.STRING)
                                                .description("상태"),
                                        fieldWithPath("message").type(JsonFieldType.STRING)
                                                .description("메시지"),
                                        fieldWithPath("data.id").type(JsonFieldType.NUMBER)
                                                .description("예약 아이디"),
                                        fieldWithPath("data.startTime").type(JsonFieldType.STRING)
                                                .description("예상 시작시간"),
                                        fieldWithPath("data.endTime").type(JsonFieldType.STRING)
                                                .description("예상 종료시간"),
                                        fieldWithPath("data.status").type(JsonFieldType.STRING)
                                                .description("예약 상태"),
                                        fieldWithPath("data.car.id").type(JsonFieldType.NUMBER)
                                                .description("차량 아이디"),
                                        fieldWithPath("data.car.id").type(JsonFieldType.NUMBER)
                                                .description("차량 아이디"),
                                        fieldWithPath("data.car.number").type(JsonFieldType.STRING)
                                                .description("차량 번호"),
                                        fieldWithPath("data.car.nickname").type(JsonFieldType.STRING)
                                                .description("차량 별명"),
                                        fieldWithPath("data.car.carType.id").type(JsonFieldType.NUMBER)
                                                .description("차종 아이디"),
                                        fieldWithPath("data.car.carType.name").type(JsonFieldType.STRING)
                                                .description("차종 이름"),
                                        fieldWithPath("data.car.carType.image").type(JsonFieldType.STRING)
                                                .description("차종 이미지 아이디"))
                                .build())));

    }

    @DisplayName("예약을 확정한다.")
    @Test
    void confirmReservation() throws Exception {
        // given
        CarType carType = CarType.builder()
                .id(1L)
                .name("아이오닉")
                .image("image")
                .build();

        Car car = Car.builder()
                .id(1L)
                .carType(carType)
                .number("1234")
                .nickname("nickname")
                .build();

        ReservationResponse response = ReservationResponse.builder()
                .id(1L)
                .car(CarResponse.from(car))
                .startTime(LocalDateTime.of(2021, 1, 1, 0, 0))
                .endTime(LocalDateTime.of(2021, 1, 1, 2, 0))
                .status(ReservationStatus.WAITING.name())
                .build();

        given(reservationService.confirm(any(), any())).willReturn(Reservation.builder().id(1L).build());
        given(reservationService.getReservation(any())).willReturn(response);

        // when
        ResultActions perform = mockMvc.perform(
                patch("/api/v1/reservations/{reservationId}", 1)
                        .header("Authorization", "Bearer token")
                        .contentType(MediaType.APPLICATION_JSON));

        // then
        perform
                .andDo(print())
                .andExpect(status().isOk())
                .andDo(document("reservation-confirm",
                        preprocessResponse(prettyPrint()),
                        resource(ResourceSnippetParameters.builder()
                                .tag("Reservation")
                                .summary("예약 확정")
                                .requestHeaders(
                                        headerWithName("Authorization").description("JWT 토큰 (Bearer)")
                                )
                                .responseFields(
                                        fieldWithPath("code").type(JsonFieldType.NUMBER)
                                                .description("코드"),
                                        fieldWithPath("status").type(JsonFieldType.STRING)
                                                .description("상태"),
                                        fieldWithPath("message").type(JsonFieldType.STRING)
                                                .description("메시지"),
                                        fieldWithPath("data.id").type(JsonFieldType.NUMBER)
                                                .description("예약 아이디"),
                                        fieldWithPath("data.startTime").type(JsonFieldType.STRING)
                                                .description("예상 시작시간"),
                                        fieldWithPath("data.endTime").type(JsonFieldType.STRING)
                                                .description("예상 종료시간"),
                                        fieldWithPath("data.status").type(JsonFieldType.STRING)
                                                .description("예약 상태"),
                                        fieldWithPath("data.car.id").type(JsonFieldType.NUMBER)
                                                .description("차량 아이디"),
                                        fieldWithPath("data.car.id").type(JsonFieldType.NUMBER)
                                                .description("차량 아이디"),
                                        fieldWithPath("data.car.number").type(JsonFieldType.STRING)
                                                .description("차량 번호"),
                                        fieldWithPath("data.car.nickname").type(JsonFieldType.STRING)
                                                .description("차량 별명"),
                                        fieldWithPath("data.car.carType.id").type(JsonFieldType.NUMBER)
                                                .description("차종 아이디"),
                                        fieldWithPath("data.car.carType.name").type(JsonFieldType.STRING)
                                                .description("차종 이름"),
                                        fieldWithPath("data.car.carType.image").type(JsonFieldType.STRING)
                                                .description("차종 이미지 아이디"))
                                .build())));

    }

}

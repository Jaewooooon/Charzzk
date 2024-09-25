package com.ssafy.charzzk.docs.reservations;

import com.epages.restdocs.apispec.ResourceSnippetParameters;
import com.ssafy.charzzk.api.controller.reservation.ReservationController;
import com.ssafy.charzzk.api.service.reservation.ReservationService;
import com.ssafy.charzzk.api.service.response.ReservationCheckTimeResponse;
import com.ssafy.charzzk.docs.RestDocsSupport;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.springframework.http.MediaType;
import org.springframework.restdocs.payload.JsonFieldType;
import org.springframework.test.web.servlet.ResultActions;

import java.time.LocalDateTime;

import static com.epages.restdocs.apispec.MockMvcRestDocumentationWrapper.document;
import static com.epages.restdocs.apispec.ResourceDocumentation.resource;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.BDDMockito.given;
import static org.mockito.Mockito.mock;
import static org.springframework.restdocs.mockmvc.RestDocumentationRequestBuilders.get;
import static org.springframework.restdocs.operation.preprocess.Preprocessors.preprocessResponse;
import static org.springframework.restdocs.operation.preprocess.Preprocessors.prettyPrint;
import static org.springframework.restdocs.payload.PayloadDocumentation.fieldWithPath;
import static org.springframework.restdocs.request.RequestDocumentation.parameterWithName;
import static org.springframework.test.web.servlet.result.MockMvcResultHandlers.print;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.status;

public class ReservationControllerDocsTest extends RestDocsSupport {

    private final ReservationService reservationService = mock(ReservationService.class);

    @Override
    protected Object initController() {
        return new ReservationController(reservationService);
    }

    @DisplayName("예약 가능한 시간을 조회한다.")
    @Test
    void checkTime() throws Exception {
        // given
        ReservationCheckTimeResponse response = ReservationCheckTimeResponse.builder()
                .chargerId(1L)
                .startTime(LocalDateTime.of(2021, 8, 1, 0, 0, 0))
                .endTime(LocalDateTime.of(2021, 8, 1, 2, 0, 0))
                .isPossibleNow(true)
                .build();

        given(reservationService.checkTime(anyLong(), anyLong(), anyBoolean(), anyInt(), any())).willReturn(response);

        // when
        ResultActions perform = mockMvc.perform(get("/api/v1/reservations/check-time")
                .param("parkingLotId", "1")
                .param("carId", "1")
                .param("fullCharge", "true")
                .param("time", "0")
                .contentType(MediaType.APPLICATION_JSON));

        // then
        perform
                .andDo(print())
                .andExpect(status().isOk())
                .andDo(document("reservations-check-time",
                        preprocessResponse(prettyPrint()),
                        resource(ResourceSnippetParameters.builder()
                                .tag("Reservation")
                                .summary("예약 가능한 시간을 조회")
                                .queryParameters(
                                        parameterWithName("parkingLotId").description("주차장 아이디"),
                                        parameterWithName("carId").description("차량 아이디"),
                                        parameterWithName("fullCharge").description("풀충전 여부"),
                                        parameterWithName("time").description("충전 시간")
                                )
                                .responseFields(
                                        fieldWithPath("code").type(JsonFieldType.NUMBER)
                                                .description("코드"),
                                        fieldWithPath("status").type(JsonFieldType.STRING)
                                                .description("상태"),
                                        fieldWithPath("message").type(JsonFieldType.STRING)
                                                .description("메시지"),
                                        fieldWithPath("data.chargerId").type(JsonFieldType.NUMBER)
                                                .description("충전기 아이디"),
                                        fieldWithPath("data.startTime").type(JsonFieldType.STRING)
                                                .description("예상 시작 시간"),
                                        fieldWithPath("data.endTime").type(JsonFieldType.STRING)
                                                .description("예상 종료 시간"),
                                        fieldWithPath("data.possibleNow").type(JsonFieldType.BOOLEAN)
                                                .description("즉시 충전 가능 여부")
                                )
                                .build())));
    }
}

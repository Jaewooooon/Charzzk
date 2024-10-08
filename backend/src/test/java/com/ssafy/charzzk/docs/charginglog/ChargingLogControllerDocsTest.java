package com.ssafy.charzzk.docs.charginglog;

import com.epages.restdocs.apispec.ResourceSnippetParameters;
import com.ssafy.charzzk.api.controller.charginglog.ChargingLogController;
import com.ssafy.charzzk.api.service.car.response.CarChargingLogResponse;
import com.ssafy.charzzk.api.service.car.response.CarTypeChargingLogResponse;
import com.ssafy.charzzk.api.service.charginglog.ChargingLogService;
import com.ssafy.charzzk.api.service.charginglog.response.ChargingLogResponse;
import com.ssafy.charzzk.docs.RestDocsSupport;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.springframework.http.MediaType;
import org.springframework.restdocs.payload.JsonFieldType;
import org.springframework.security.test.context.support.WithMockUser;
import org.springframework.test.web.servlet.ResultActions;

import java.time.LocalDateTime;
import java.util.List;

import static com.epages.restdocs.apispec.MockMvcRestDocumentationWrapper.document;
import static com.epages.restdocs.apispec.ResourceDocumentation.resource;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.BDDMockito.given;
import static org.mockito.Mockito.mock;
import static org.springframework.restdocs.mockmvc.RestDocumentationRequestBuilders.get;
import static org.springframework.restdocs.operation.preprocess.Preprocessors.preprocessResponse;
import static org.springframework.restdocs.operation.preprocess.Preprocessors.prettyPrint;
import static org.springframework.restdocs.payload.PayloadDocumentation.fieldWithPath;
import static org.springframework.test.web.servlet.result.MockMvcResultHandlers.print;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.status;


class ChargingLogControllerDocsTest extends RestDocsSupport {

    private final ChargingLogService chargingLogService = mock(ChargingLogService.class);

    @Override
    protected Object initController() {
        return new ChargingLogController(chargingLogService);
    }

    @DisplayName("충전 로그 목록 조회에 성공한다.")
    @WithMockUser
    @Test
    void getChargingLogList() throws Exception {

        // given
        CarTypeChargingLogResponse carType = CarTypeChargingLogResponse.builder()
                .id(1L)
                .name("차종1")
                .build();

        CarChargingLogResponse car = CarChargingLogResponse.builder()
                .id(1L)
                .carType(carType)
                .number("차 번호판")
                .nickname("차 닉네임")
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
                        .contentType(MediaType.APPLICATION_JSON)
        );

        // then
        perform.andDo(print())
                .andExpect(status().isOk())
                .andDo(document("charging-log-list-get",
                        preprocessResponse(prettyPrint()),
                        resource(ResourceSnippetParameters.builder()
                                .tag("ChargingLog")
                                .summary("충전 로그 목록 조회")
                                .description("유저의 모든 충전 로그를 조회한다.")
                                .responseFields(
                                        fieldWithPath("code").type(JsonFieldType.NUMBER)
                                                .description("응답 코드"),
                                        fieldWithPath("status").type(JsonFieldType.STRING)
                                                .description("응답 상태"),
                                        fieldWithPath("message").type(JsonFieldType.STRING).description("응답 메시지"),
                                        fieldWithPath("data[].id").type(JsonFieldType.NUMBER)
                                                .description("충전 로그 ID"),
                                        fieldWithPath("data[].car.id").type(JsonFieldType.NUMBER)
                                                .description("차량 ID"),
                                        fieldWithPath("data[].car.carType.id").type(JsonFieldType.NUMBER)
                                                .description("차종 ID"),
                                        fieldWithPath("data[].car.carType.name").type(JsonFieldType.STRING)
                                                .description("차종 이름"),
                                        fieldWithPath("data[].car.number").type(JsonFieldType.STRING)
                                                .description("차량 번호"),
                                        fieldWithPath("data[].car.nickname").type(JsonFieldType.STRING)
                                                .description("차량 닉네임"),
                                        fieldWithPath("data[].startTime").type(JsonFieldType.STRING)
                                                .description("충전 시작 시간"),
                                        fieldWithPath("data[].endTime").type(JsonFieldType.STRING)
                                                .description("충전 종료 시간"),
                                        fieldWithPath("data[].chargeAmount").type(JsonFieldType.NUMBER)
                                                .description("충전량"),
                                        fieldWithPath("data[].chargeCost").type(JsonFieldType.NUMBER)
                                                .description("충전 요금")
                                )
                                .build()
                        )
                ));
    }
}

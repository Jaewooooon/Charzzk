package com.ssafy.charzzk.docs.packinglot;

import com.epages.restdocs.apispec.ResourceSnippetParameters;
import com.ssafy.charzzk.api.controller.parkinglot.ParkingLotController;
import com.ssafy.charzzk.api.controller.parkinglot.request.ParkingLotListRequest;
import com.ssafy.charzzk.api.service.parkinglot.ParkingLotService;
import com.ssafy.charzzk.api.service.parkinglot.response.ParkingLotListResponse;
import com.ssafy.charzzk.api.service.parkinglot.response.ParkingLotResponse;
import com.ssafy.charzzk.api.service.parkinglot.response.ParkingSpotListResponse;
import com.ssafy.charzzk.docs.RestDocsSupport;
import com.ssafy.charzzk.domain.parkinglot.Location;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.springframework.http.MediaType;
import org.springframework.restdocs.payload.JsonFieldType;
import org.springframework.test.web.servlet.ResultActions;

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
import static org.springframework.security.test.web.servlet.request.SecurityMockMvcRequestPostProcessors.csrf;
import static org.springframework.test.web.servlet.result.MockMvcResultHandlers.print;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.status;

class ParkingLotControllerDocsTest extends RestDocsSupport {

    private final ParkingLotService parkingLotService = mock(ParkingLotService.class);

    @Override
    protected Object initController() {
        return new ParkingLotController(parkingLotService);
    }

    @DisplayName("주변 주차장 리스트 현재 거리와 가까운순 오름차순으로 조회한다.")
    @Test
    void getParkingLotList() throws Exception {
        // given
        ParkingLotListRequest request = ParkingLotListRequest.builder()
                .latitude(37.123456)
                .longitude(127.123456)
                .build();

        Location location = Location.builder()
                .latitude(0.0)
                .longitude(0.0)
                .build();

        List<ParkingLotListResponse> response = List.of(
                ParkingLotListResponse.builder()
                        .id(1L)
                        .name("주차장1")
                        .location(location)
                        .image("주차장 이미지")
                        .distance(100.0)
                        .build()
        );

        given(parkingLotService.getParkingLotList(any())).willReturn(response);

        // when
        ResultActions perform = mockMvc.perform(
                get("/api/v1/parking-lot")
                        .with(csrf())
                        .content(objectMapper.writeValueAsString(request))
                        .contentType(MediaType.APPLICATION_JSON)
        );

        // then
        perform
                .andDo(print())
                .andExpect(status().isOk())
                .andDo(document("parking-lot-list-get",
                        preprocessResponse(prettyPrint()),
                        resource(ResourceSnippetParameters.builder()
                                .tag("ParkingLot")
                                .summary("주차장 목록 조회")
                                .requestFields(
                                        fieldWithPath("latitude").type(JsonFieldType.NUMBER)
                                                .description("위도"),
                                        fieldWithPath("longitude").type(JsonFieldType.NUMBER)
                                                .description("경도")
                                )
                                .responseFields(
                                        fieldWithPath("code").type(JsonFieldType.NUMBER)
                                                .description("코드"),
                                        fieldWithPath("status").type(JsonFieldType.STRING)
                                                .description("상태"),
                                        fieldWithPath("message").type(JsonFieldType.STRING)
                                                .description("메시지"),
                                        fieldWithPath("data[].id").type(JsonFieldType.NUMBER)
                                                .description("주차장 아이디"),
                                        fieldWithPath("data[].name").type(JsonFieldType.STRING)
                                                .description("주차장 이름"),
                                        fieldWithPath("data[].location").type(JsonFieldType.OBJECT)
                                                .description("좌표"),
                                        fieldWithPath("data[].location.latitude").type(JsonFieldType.NUMBER)
                                                .description("위도"),
                                        fieldWithPath("data[].location.longitude").type(JsonFieldType.NUMBER)
                                                .description("경도"),
                                        fieldWithPath("data[].image").type(JsonFieldType.STRING)
                                                .description("주차장 이미지"),
                                        fieldWithPath("data[].distance").type(JsonFieldType.NUMBER)
                                                .description("주차장까지 거리")
                                )
                                .build())));

    }

    @DisplayName("주차장 상세정보를 조회한다.")
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
        );

        // then
        perform
                .andDo(print())
                .andExpect(status().isOk())
                .andDo(document("parking-lot-get",
                        preprocessResponse(prettyPrint()),
                        resource(ResourceSnippetParameters.builder()
                                .tag("ParkingLot")
                                .summary("주차장 상세 조회")
                                .responseFields(
                                        fieldWithPath("code").type(JsonFieldType.NUMBER)
                                                .description("코드"),
                                        fieldWithPath("status").type(JsonFieldType.STRING)
                                                .description("상태"),
                                        fieldWithPath("message").type(JsonFieldType.STRING)
                                                .description("메시지"),
                                        fieldWithPath("data.id").type(JsonFieldType.NUMBER)
                                                .description("주차장 아이디"),
                                        fieldWithPath("data.name").type(JsonFieldType.STRING)
                                                .description("주차장 이름"),
                                        fieldWithPath("data.parkingMapImage").type(JsonFieldType.STRING)
                                                .description("주차칸 이미지"),
                                        fieldWithPath("data.parkingSpots[]").type(JsonFieldType.ARRAY)
                                                .description("주차칸 이미지"),
                                        fieldWithPath("data.parkingSpots[].id").type(JsonFieldType.NUMBER)
                                                .description("주차칸 아이디"),
                                        fieldWithPath("data.parkingSpots[].name").type(JsonFieldType.STRING)
                                                .description("주차칸 이름")
                                )
                                .build())));

    }

}
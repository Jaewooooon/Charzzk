package com.ssafy.charzzk.docs.car;

import com.epages.restdocs.apispec.ResourceSnippetParameters;
import com.ssafy.charzzk.api.controller.car.CarController;
import com.ssafy.charzzk.api.controller.car.request.CarRequest;
import com.ssafy.charzzk.api.service.car.CarService;
import com.ssafy.charzzk.api.service.car.response.CarListResponse;
import com.ssafy.charzzk.api.service.car.response.CarResponse;
import com.ssafy.charzzk.api.service.car.response.CarTypeResponse;
import com.ssafy.charzzk.docs.RestDocsSupport;
import com.ssafy.charzzk.domain.car.CarType;
import com.ssafy.charzzk.domain.user.User;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.springframework.http.MediaType;
import org.springframework.restdocs.payload.JsonFieldType;
import org.springframework.test.web.servlet.ResultActions;

import java.time.LocalDateTime;
import java.util.List;

import static com.epages.restdocs.apispec.MockMvcRestDocumentationWrapper.document;
import static com.epages.restdocs.apispec.ResourceDocumentation.resource;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.BDDMockito.given;
import static org.mockito.Mockito.mock;
import static org.springframework.restdocs.headers.HeaderDocumentation.headerWithName;
import static org.springframework.restdocs.mockmvc.RestDocumentationRequestBuilders.*;
import static org.springframework.restdocs.operation.preprocess.Preprocessors.preprocessResponse;
import static org.springframework.restdocs.operation.preprocess.Preprocessors.prettyPrint;
import static org.springframework.restdocs.payload.PayloadDocumentation.fieldWithPath;
import static org.springframework.restdocs.request.RequestDocumentation.parameterWithName;
import static org.springframework.test.web.servlet.result.MockMvcResultHandlers.print;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.status;

public class CarControllerDocsTest extends RestDocsSupport {

    private final CarService carService = mock(CarService.class);

    @Override
    protected Object initController() {
        return new CarController(carService);
    }

    @DisplayName("차량을 등록한다")
    @Test
    public void createCar() throws Exception {
        // given
        CarRequest request = CarRequest.builder()
                .carTypeId(1L)
                .number("12가1234")
                .nickname("붕붕이")
                .build();

        CarType carType = CarType.builder()
                .id(1L)
                .name("소형차")
                .image("image")
                .build();

        CarResponse response = CarResponse.builder()
                .id(1L)
                .carType(carType)
                .number("12가1234")
                .nickname("붕붕이")
                .build();

        given(carService.getCar(any())).willReturn(response);

        // when
        ResultActions perform = mockMvc.perform(post("/api/v1/cars")
                .header("Authorization", "Bearer token")
                .content(objectMapper.writeValueAsString(request))
                .contentType(MediaType.APPLICATION_JSON));

        // then
        perform
                .andDo(print())
                .andExpect(status().isOk())
                .andDo(document("car-create",
                        preprocessResponse(prettyPrint()),
                        resource(ResourceSnippetParameters.builder()
                                .tag("Car")
                                .summary("차량 등록")
                                .requestHeaders(
                                        headerWithName("Authorization").description("JWT 토큰 (Bearer)")
                                )
                                .requestFields(
                                        fieldWithPath("carTypeId").type(JsonFieldType.NUMBER)
                                                .description("차종 아이디"),
                                        fieldWithPath("number").type(JsonFieldType.STRING)
                                                .description("차량 번호"),
                                        fieldWithPath("nickname").type(JsonFieldType.STRING)
                                                .optional().description("차량 별명")
                                )
                                .responseFields(
                                        fieldWithPath("code").type(JsonFieldType.NUMBER)
                                                .description("코드"),
                                        fieldWithPath("status").type(JsonFieldType.STRING)
                                                .description("상태"),
                                        fieldWithPath("message").type(JsonFieldType.STRING)
                                                .description("메시지"),
                                        fieldWithPath("data.id").type(JsonFieldType.NUMBER)
                                                .description("차량 아이디"),
                                        fieldWithPath("data.number").type(JsonFieldType.STRING)
                                                .description("차량 번호"),
                                        fieldWithPath("data.nickname").type(JsonFieldType.STRING)
                                                .description("차량 별명"),
                                        fieldWithPath("data.carType.id").type(JsonFieldType.NUMBER)
                                                .description("차종 아이디"),
                                        fieldWithPath("data.carType.name").type(JsonFieldType.STRING)
                                                .description("차종"),
                                        fieldWithPath("data.carType.image").type(JsonFieldType.STRING)
                                                .description("차량 이미지")
                                )
                                .build())));

    }

    @DisplayName("차량 종류를 조회한다")
    @Test
    public void getCarTypes() throws Exception {
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

        List<CarTypeResponse> carTypeResponses = List.of(carType1, carType2);

        given(carService.getCarTypes(any())).willReturn(carTypeResponses);

        // when
        ResultActions perform = mockMvc.perform(get("/api/v1/car-types")
                .param("q", "테슬라"));

        // then
        perform
                .andDo(print())
                .andExpect(status().isOk())
                .andDo(document("car-types",
                        preprocessResponse(prettyPrint()),
                        resource(ResourceSnippetParameters.builder()
                                .tag("Car")
                                .summary("차량 종류 조회")
                                .queryParameters(
                                        parameterWithName("q").optional().description("검색어")
                                )
                                .responseFields(
                                        fieldWithPath("code").type(JsonFieldType.NUMBER)
                                                .description("코드"),
                                        fieldWithPath("status").type(JsonFieldType.STRING)
                                                .description("상태"),
                                        fieldWithPath("message").type(JsonFieldType.STRING)
                                                .description("메시지"),
                                        fieldWithPath("data[].id").type(JsonFieldType.NUMBER)
                                                .description("차종 아이디"),
                                        fieldWithPath("data[].name").type(JsonFieldType.STRING)
                                                .description("차종 이름"),
                                        fieldWithPath("data[].image").type(JsonFieldType.STRING)
                                                .description("차종 이미지")
                                )
                                .build())));

    }

    @DisplayName("차량 정보를 수정한다")
    @Test
    public void updateCar() throws Exception {
        // given
        CarRequest request = CarRequest.builder()
                .carTypeId(1L)
                .number("12가1234")
                .nickname("붕붕이")
                .build();

        CarType carType = CarType.builder()
                .id(1L)
                .name("소형차")
                .image("image")
                .build();

        CarResponse response = CarResponse.builder()
                .id(1L)
                .carType(carType)
                .number("12가1234")
                .nickname("붕붕이")
                .build();

        given(carService.getCar(any())).willReturn(response);

        // when
        ResultActions perform = mockMvc.perform(patch("/api/v1/cars/{carId}", 1L)
                .content(objectMapper.writeValueAsString(request))
                .contentType(MediaType.APPLICATION_JSON));

        // then
        perform
                .andDo(print())
                .andExpect(status().isOk())
                .andDo(document("car-update",
                        preprocessResponse(prettyPrint()),
                        resource(ResourceSnippetParameters.builder()
                                .tag("Car")
                                .summary("차량 정보 수정")
                                .pathParameters(
                                        parameterWithName("carId").description("차량 아이디")
                                )
                                .requestFields(
                                        fieldWithPath("carTypeId").type(JsonFieldType.NUMBER)
                                                .description("차종 아이디"),
                                        fieldWithPath("number").type(JsonFieldType.STRING)
                                                .description("차량 번호"),
                                        fieldWithPath("nickname").type(JsonFieldType.STRING)
                                                .optional().description("차량 별명")
                                )
                                .responseFields(
                                        fieldWithPath("code").type(JsonFieldType.NUMBER)
                                                .description("코드"),
                                        fieldWithPath("status").type(JsonFieldType.STRING)
                                                .description("상태"),
                                        fieldWithPath("message").type(JsonFieldType.STRING)
                                                .description("메시지"),
                                        fieldWithPath("data.id").type(JsonFieldType.NUMBER)
                                                .description("차량 아이디"),
                                        fieldWithPath("data.number").type(JsonFieldType.STRING)
                                                .description("차량 번호"),
                                        fieldWithPath("data.nickname").type(JsonFieldType.STRING)
                                                .description("차량 별명"),
                                        fieldWithPath("data.carType.id").type(JsonFieldType.NUMBER)
                                                .description("차종 아이디"),
                                        fieldWithPath("data.carType.name").type(JsonFieldType.STRING)
                                                .description("차종 이름"),
                                        fieldWithPath("data.carType.image").type(JsonFieldType.STRING)
                                                .description("차종 이미지"))
                                .build())));

    }

    @DisplayName("차량을 삭제한다")
    @Test
    public void deleteCar() throws Exception {
        // when
        ResultActions perform = mockMvc.perform(delete("/api/v1/cars/{carId}", 1L));

        // then
        perform
                .andDo(print())
                .andExpect(status().isOk())
                .andDo(document("car-delete",
                        preprocessResponse(prettyPrint()),
                        resource(ResourceSnippetParameters.builder()
                                .tag("Car")
                                .summary("차량 삭제")
                                .pathParameters(
                                        parameterWithName("carId").description("차량 아이디")
                                )
                                .responseFields(
                                        fieldWithPath("code").type(JsonFieldType.NUMBER)
                                                .description("코드"),
                                        fieldWithPath("status").type(JsonFieldType.STRING)
                                                .description("상태"),
                                        fieldWithPath("message").type(JsonFieldType.STRING)
                                                .description("메시지"),
                                        fieldWithPath("data").type(JsonFieldType.NULL)
                                                .description("null")
                                )
                                .build())));
    }

    @DisplayName("사용자의 차량 목록을 조회한다.")
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
                .chargeCost(150000L)
                .chargeAmount(500.1)
                .build();

        CarListResponse carResponse2 = CarListResponse.builder()
                .id(2L)
                .carType(carTypeResponse)
                .number("22나2222")
                .nickname("순이")
                .isCharging(true)
                .chargeCost(300000L)
                .chargeAmount(1000.2)
                .build();

        List<CarListResponse> carList = List.of(carResponse1, carResponse2);

        LocalDateTime startOfMonth = LocalDateTime.of(2024, 9, 1, 0, 0, 0);
        LocalDateTime endOfMonth = LocalDateTime.of(2024, 9, 30, 23, 59, 59);

        given(carService.getCarList(any(User.class), any(LocalDateTime.class), any(LocalDateTime.class))).willReturn(carList);

        // when
        ResultActions perform = mockMvc.perform(
                get("/api/v1/cars/me")
                        .header("Authorization", "Bearer token")
                        .contentType(MediaType.APPLICATION_JSON)
        );

        // then
        perform
                .andDo(print())
                .andExpect(status().isOk())
                .andDo(document("car-list",
                        preprocessResponse(prettyPrint()),
                        resource(ResourceSnippetParameters.builder()
                                .tag("Car")
                                .summary("내 차량 목록 조회")
                                .description("로그인한 사용자의 차량 목록을 조회한다.")
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
                                        fieldWithPath("data").type(JsonFieldType.ARRAY)
                                                .description("차량 목록"),
                                        fieldWithPath("data[].id").type(JsonFieldType.NUMBER)
                                                .description("차량 ID"),
                                        fieldWithPath("data[].number").type(JsonFieldType.STRING)
                                                .description("차량 번호"),
                                        fieldWithPath("data[].nickname").type(JsonFieldType.STRING)
                                                .optional().description("차량 별명"),
                                        fieldWithPath("data[].charging").type(JsonFieldType.BOOLEAN)
                                                .description("충전 중 여부"),
                                        fieldWithPath("data[].chargeCost").type(JsonFieldType.NUMBER)
                                                .description("이번달 충전 비용"),
                                        fieldWithPath("data[].chargeAmount").type(JsonFieldType.NUMBER)
                                                .description("이번달 충전 양"),
                                        fieldWithPath("data[].carType.id").type(JsonFieldType.NUMBER)
                                                .description("차종 아이디"),
                                        fieldWithPath("data[].carType.name").type(JsonFieldType.STRING)
                                                .description("차종 이름"),
                                        fieldWithPath("data[].carType.image").type(JsonFieldType.STRING)
                                                .description("차종 이미지")
                                )
                                .build()
                        )));
    }

}

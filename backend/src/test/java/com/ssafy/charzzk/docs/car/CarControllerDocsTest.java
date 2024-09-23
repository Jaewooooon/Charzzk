package com.ssafy.charzzk.docs.car;

import com.epages.restdocs.apispec.ResourceSnippetParameters;
import com.ssafy.charzzk.api.controller.car.CarController;
import com.ssafy.charzzk.api.controller.car.request.CarRequest;
import com.ssafy.charzzk.api.service.car.CarService;
import com.ssafy.charzzk.api.service.car.response.CarResponse;
import com.ssafy.charzzk.api.service.car.response.CarTypeResponse;
import com.ssafy.charzzk.docs.RestDocsSupport;
import com.ssafy.charzzk.domain.car.CarType;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.springframework.http.MediaType;
import org.springframework.restdocs.payload.JsonFieldType;
import org.springframework.test.web.servlet.ResultActions;

import java.util.Arrays;
import java.util.List;

import static com.epages.restdocs.apispec.MockMvcRestDocumentationWrapper.document;
import static com.epages.restdocs.apispec.ResourceDocumentation.resource;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.BDDMockito.given;
import static org.mockito.Mockito.mock;
import static org.springframework.restdocs.mockmvc.RestDocumentationRequestBuilders.post;
import static org.springframework.restdocs.operation.preprocess.Preprocessors.preprocessResponse;
import static org.springframework.restdocs.operation.preprocess.Preprocessors.prettyPrint;
import static org.springframework.restdocs.payload.PayloadDocumentation.fieldWithPath;
import static org.springframework.restdocs.request.RequestDocumentation.parameterWithName;
import static org.springframework.test.web.servlet.request.MockMvcRequestBuilders.get;
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

}

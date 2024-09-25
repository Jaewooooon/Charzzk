package com.ssafy.charzzk.docs.report;

import com.epages.restdocs.apispec.ResourceSnippetParameters;
import com.ssafy.charzzk.api.controller.report.ReportController;
import com.ssafy.charzzk.api.controller.report.request.ReportRequest;
import com.ssafy.charzzk.api.service.parkinglot.response.ParkingLotReportResponse;
import com.ssafy.charzzk.api.service.report.ReportService;
import com.ssafy.charzzk.api.service.report.response.ReportResponse;
import com.ssafy.charzzk.api.service.user.response.UserResponse;
import com.ssafy.charzzk.docs.RestDocsSupport;
import com.ssafy.charzzk.domain.report.ReportType;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.springframework.http.MediaType;
import org.springframework.mock.web.MockMultipartFile;
import org.springframework.restdocs.payload.JsonFieldType;
import org.springframework.test.web.servlet.ResultActions;

import java.time.LocalDateTime;

import static com.epages.restdocs.apispec.MockMvcRestDocumentationWrapper.document;
import static com.epages.restdocs.apispec.ResourceDocumentation.resource;
import static org.mockito.ArgumentMatchers.anyLong;
import static org.mockito.BDDMockito.given;
import static org.mockito.Mockito.mock;
import static org.springframework.restdocs.headers.HeaderDocumentation.headerWithName;
import static org.springframework.restdocs.mockmvc.RestDocumentationRequestBuilders.multipart;
import static org.springframework.restdocs.operation.preprocess.Preprocessors.preprocessResponse;
import static org.springframework.restdocs.operation.preprocess.Preprocessors.prettyPrint;
import static org.springframework.restdocs.payload.PayloadDocumentation.fieldWithPath;
import static org.springframework.test.web.servlet.result.MockMvcResultHandlers.print;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.status;
import static org.springframework.restdocs.request.RequestDocumentation.partWithName;
import static org.springframework.restdocs.request.RequestDocumentation.requestParts;


public class ReportControllerDocsTest extends RestDocsSupport {

    private final ReportService reportService = mock(ReportService.class);

    @Override
    protected Object initController() {
        return new ReportController(reportService);
    }

    @DisplayName("신고를 한다.")
    @Test
    public void createReport() throws Exception {
        // given
        ReportRequest reportRequest = ReportRequest.builder()
                .serialNumber("1234ABCD")
                .type(ReportType.FLIPPED)
                .content("로봇이 뒤집혔습니다.")
                .build();

        UserResponse userResponse = UserResponse.builder()
                .id(1L)
                .username("testuser@gmail.com")
                .nickname("유저1")
                .build();

        ParkingLotReportResponse  parkingLotReportResponse = ParkingLotReportResponse.builder()
                .id(1L)
                .name("장덕 공영 주차장")
                .build();

        ReportResponse reportResponse = ReportResponse.builder()
                .id(1L)
                .user(userResponse)
                .reportType(ReportType.FLIPPED)
                .parkingLot(parkingLotReportResponse)
                .content("로봇이 뒤집혔습니다.")
                .image("https://mockurl.com/test-image.jpg")
                .isRead(false)
                .createdAt(LocalDateTime.now())
                .build();

        given(reportService.getReport(anyLong())).willReturn(reportResponse);

        // report 객체를 JSON 문자열로 변환하여 multipart 파일로 생성
        MockMultipartFile reportPart = new MockMultipartFile(
                "report", // @RequestPart("report")와 일치해야 함
                "",
                "application/json",
                objectMapper.writeValueAsBytes(reportRequest)
        );

        // image 파일 생성
        MockMultipartFile imagePart = new MockMultipartFile(
                "image",
                "test-image.jpg",
                "image/jpeg",
                "test-image-content".getBytes()
        );

        // when
        ResultActions perform = mockMvc.perform(
                multipart("/api/v1/reports")
                        .file(reportPart)
                        .file(imagePart)
                        .header("Authorization", "Bearer token")
                        .contentType(MediaType.MULTIPART_FORM_DATA)
        );


        // then
        perform
                .andDo(print())
                .andExpect(status().isOk())
                .andDo(document("report-create",
                        preprocessResponse(prettyPrint()),
                        resource(ResourceSnippetParameters.builder()
                                .tag("Report")
                                .summary("신고하기")
                                .description("사용자가 로봇 관련 문제를 신고하는 기능")
                                .requestHeaders(
                                        headerWithName("Authorization").description("JWT 토큰 (Bearer)")
                                )
                                .responseFields(
                                        fieldWithPath("code").type(JsonFieldType.NUMBER)
                                                .description("응답 코드"),
                                        fieldWithPath("status").type(JsonFieldType.STRING)
                                                .description("응답 상태"),
                                        fieldWithPath("message").type(JsonFieldType.STRING)
                                                .description("응답 메시지"),
                                        fieldWithPath("data.id").type(JsonFieldType.NUMBER)
                                                .description("신고 아이디"),
                                        fieldWithPath("data.user.id").type(JsonFieldType.NUMBER)
                                                .description("유저 아이디"),
                                        fieldWithPath("data.user.username").type(JsonFieldType.STRING)
                                                .description("유저 네임"),
                                        fieldWithPath("data.user.nickname").type(JsonFieldType.STRING)
                                                .description("유저 닉네임"),
                                        fieldWithPath("data.reportType").type(JsonFieldType.STRING)
                                                .description("신고 유형"),
                                        fieldWithPath("data.parkingLot.id").type(JsonFieldType.NUMBER)
                                                .description("주차장 ID"),
                                        fieldWithPath("data.parkingLot.name").type(JsonFieldType.STRING)
                                                .description("주차장 이름"),
                                        fieldWithPath("data.content").type(JsonFieldType.STRING)
                                                .description("신고 내용"),
                                        fieldWithPath("data.image").type(JsonFieldType.STRING)
                                                .description("신고 이미지 URL"),
                                        fieldWithPath("data.createdAt").type(JsonFieldType.STRING)
                                                .description("신고 시각"),
                                        fieldWithPath("data.read").type(JsonFieldType.BOOLEAN)
                                                .description("신고 확인 여부")
                                )
                                .build()
                        ),

                        requestParts(
                                partWithName("report").description("신고 정보"),
                                partWithName("image").description("신고 이미지 파일")
                        )
                ));
    }
}

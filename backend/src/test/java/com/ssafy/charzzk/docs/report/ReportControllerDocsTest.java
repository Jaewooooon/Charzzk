package com.ssafy.charzzk.docs.report;

import com.epages.restdocs.apispec.ResourceSnippetParameters;
import com.ssafy.charzzk.api.controller.report.ReportController;
import com.ssafy.charzzk.api.controller.report.request.ReportRequest;
import com.ssafy.charzzk.api.service.parkinglot.response.ParkingLotReportResponse;
import com.ssafy.charzzk.api.service.report.ReportService;
import com.ssafy.charzzk.api.service.report.response.ReportListResponse;
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
import java.util.List;

import static com.epages.restdocs.apispec.MockMvcRestDocumentationWrapper.document;
import static com.epages.restdocs.apispec.ResourceDocumentation.resource;
import static org.mockito.ArgumentMatchers.anyLong;
import static org.mockito.BDDMockito.given;
import static org.mockito.Mockito.mock;
import static org.springframework.restdocs.headers.HeaderDocumentation.headerWithName;
import static org.springframework.restdocs.mockmvc.RestDocumentationRequestBuilders.multipart;
import static org.springframework.restdocs.mockmvc.RestDocumentationRequestBuilders.patch;
import static org.springframework.restdocs.mockmvc.RestDocumentationRequestBuilders.get;
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

        ParkingLotReportResponse parkingLotReportResponse = ParkingLotReportResponse.builder()
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
                                .summary("신고 등록")
                                .description("사용자는 사진과 함께 충전 로봇에 대한 신고를 할 수 있다.")
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

    @DisplayName("신고를 읽는다.")
    @Test
    public void readReport() throws Exception {
        // given
        UserResponse userResponse = UserResponse.builder()
                .id(1L)
                .username("testuser@gmail.com")
                .nickname("유저1")
                .build();

        ParkingLotReportResponse parkingLotReportResponse = ParkingLotReportResponse.builder()
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
                .isRead(true) // 읽은 상태로 설정
                .createdAt(LocalDateTime.now())
                .build();

        given(reportService.getReport(anyLong())).willReturn(reportResponse);

        // when
        ResultActions perform = mockMvc.perform(
                patch("/api/v1/reports/{reportId}", 1L)
                        .header("Authorization", "Bearer token")
                        .contentType(MediaType.APPLICATION_JSON)
        );

        // then
        perform
                .andDo(print())
                .andExpect(status().isOk())
                .andDo(document("report-read",
                        preprocessResponse(prettyPrint()),
                        resource(ResourceSnippetParameters.builder()
                                .tag("Report")
                                .summary("신고 읽기")
                                .description("관리자는 읽지 않은 신고를 읽은 상태로 변경할 수 있다.")
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
                                ).build()
                        )
                ));
    }

    @DisplayName("신고 상세 조회를 한다.")
    @Test
    public void getReport() throws Exception {
        // given
        UserResponse userResponse = UserResponse.builder()
                .id(1L)
                .username("testuser@gmail.com")
                .nickname("유저1")
                .build();

        ParkingLotReportResponse parkingLotReportResponse = ParkingLotReportResponse.builder()
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
                .isRead(true)
                .createdAt(LocalDateTime.now())
                .build();

        given(reportService.getReport(anyLong())).willReturn(reportResponse);

        // when
        ResultActions perform = mockMvc.perform(
                get("/api/v1/reports/{reportId}", 1L)
                        .header("Authorization", "Bearer token")
                        .contentType(MediaType.APPLICATION_JSON)
        );

        // then
        perform
                .andDo(print())
                .andExpect(status().isOk())
                .andDo(document("report-get",
                        preprocessResponse(prettyPrint()),
                        resource(ResourceSnippetParameters.builder()
                                .tag("Report")
                                .summary("신고 상세 조회")
                                .description("관리자는 특정 신고의 상세 정보를 조회할 수 있다.")
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
                        )
                ));
    }

    @DisplayName("신고 목록 조회를 한다.")
    @Test
    public void getReportList() throws Exception {
        // given
        UserResponse userResponse1 = UserResponse.builder()
                .id(1L)
                .username("testuser1@gmail.com")
                .nickname("유저1")
                .build();

        UserResponse userResponse2 = UserResponse.builder()
                .id(2L)
                .username("testuser2@gmail.com")
                .nickname("유저2")
                .build();

        ParkingLotReportResponse parkingLotResponse = ParkingLotReportResponse.builder()
                .id(1L)
                .name("장덕 공영 주차장")
                .build();

        ReportListResponse reportResponse1 = ReportListResponse.builder()
                .id(1L)
                .user(userResponse1)
                .reportType(ReportType.FLIPPED)
                .parkingLot(parkingLotResponse)
                .isRead(false)
                .createdAt(LocalDateTime.now())
                .build();

        ReportListResponse reportResponse2 = ReportListResponse.builder()
                .id(2L)
                .user(userResponse2)
                .reportType(ReportType.BROKEN)
                .parkingLot(parkingLotResponse)
                .isRead(true)
                .createdAt(LocalDateTime.now())
                .build();

        given(reportService.getReportList()).willReturn(List.of(reportResponse1, reportResponse2));

        // when
        ResultActions perform = mockMvc.perform(
                get("/api/v1/reports")
                        .header("Authorization", "Bearer token")
                        .contentType(MediaType.APPLICATION_JSON)
        );

        // then
        perform.andDo(print())
                .andExpect(status().isOk())
                .andDo(document("report-get-list",
                        preprocessResponse(prettyPrint()),
                        resource(ResourceSnippetParameters.builder()
                                .tag("Report")
                                .summary("신고 목록 조회")
                                .description("신고 목록을 조회한다.")
                                .requestHeaders(
                                        headerWithName("Authorization").description("JWT")
                                )
                                .responseFields(
                                        fieldWithPath("code").type(JsonFieldType.NUMBER)
                                                .description("코드"),
                                        fieldWithPath("status").type(JsonFieldType.STRING)
                                                .description("상태"),
                                        fieldWithPath("message").type(JsonFieldType.STRING)
                                                .description("메시지"),
                                        fieldWithPath("data").type(JsonFieldType.ARRAY)
                                                .description("신고 목록"),
                                        fieldWithPath("data[].id").type(JsonFieldType.NUMBER)
                                                .description("신고 아이디"),
                                        fieldWithPath("data[].user.id").type(JsonFieldType.NUMBER)
                                                .description("유저 아이디"),
                                        fieldWithPath("data[].user.username").type(JsonFieldType.STRING)
                                                .description("유저 네임"),
                                        fieldWithPath("data[].user.nickname").type(JsonFieldType.STRING)
                                                .description("유저 닉네임"),
                                        fieldWithPath("data[].reportType").type(JsonFieldType.STRING)
                                                .description("신고 유형"),
                                        fieldWithPath("data[].parkingLot.id").type(JsonFieldType.NUMBER)
                                                .description("주차장 ID"),
                                        fieldWithPath("data[].parkingLot.name").type(JsonFieldType.STRING)
                                                .description("주차장 이름"),
                                        fieldWithPath("data[].read").type(JsonFieldType.BOOLEAN)
                                                .description("신고 확인 여부"),
                                        fieldWithPath("data[].createdAt").type(JsonFieldType.STRING)
                                                .description("신고 시각")
                                ).build()
                        )
                ));
    }
}

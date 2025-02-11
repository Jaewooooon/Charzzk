package com.ssafy.charzzk.api.controller.report;

import com.ssafy.charzzk.ControllerTestSupport;
import com.ssafy.charzzk.api.controller.report.request.ReportRequest;
import com.ssafy.charzzk.api.service.parkinglot.response.ParkingLotReportResponse;
import com.ssafy.charzzk.api.service.report.response.ReportListResponse;
import com.ssafy.charzzk.api.service.report.response.ReportResponse;
import com.ssafy.charzzk.api.service.user.response.UserResponse;
import com.ssafy.charzzk.domain.report.ReportType;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.springframework.http.MediaType;
import org.springframework.mock.web.MockMultipartFile;
import org.springframework.security.test.context.support.WithMockUser;
import org.springframework.test.web.servlet.ResultActions;

import java.time.LocalDateTime;
import java.util.List;

import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.anyLong;
import static org.mockito.BDDMockito.given;
import static org.mockito.BDDMockito.willDoNothing;
import static org.springframework.security.test.web.servlet.request.SecurityMockMvcRequestPostProcessors.csrf;
import static org.springframework.test.web.servlet.request.MockMvcRequestBuilders.multipart;
import static org.springframework.test.web.servlet.request.MockMvcRequestBuilders.patch;
import static org.springframework.test.web.servlet.request.MockMvcRequestBuilders.get;
import static org.springframework.test.web.servlet.result.MockMvcResultHandlers.print;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.jsonPath;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.status;


class ReportControllerTest extends ControllerTestSupport {

    @DisplayName("신고하기 요청하면 신고가 정상적으로 생성된다.")
    @WithMockUser
    @Test
    public void createReport() throws Exception {
        // given
        ReportRequest request = ReportRequest.builder()
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

        ReportResponse response = ReportResponse.builder()
                .id(1L)
                .user(userResponse)
                .reportType(ReportType.FLIPPED)
                .parkingLot(parkingLotReportResponse)
                .content("로봇이 뒤집혔습니다.")
                .image("https://mockurl.com/test-image.jpg")
                .isRead(false)
                .createdAt(LocalDateTime.now())
                .build();

        given(reportService.createReport(any(), any(), any())).willReturn(1L);
        given(reportService.getReport(1L)).willReturn(response);

        // 파일 업로드를 위한 MockMultipartFile 생성
        MockMultipartFile reportPart = new MockMultipartFile(
                "report",
                "",
                "application/json",
                objectMapper.writeValueAsBytes(request)
        );

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
                        .with(csrf())
                        .header("Authorization", "Bearer token")
                        .contentType(MediaType.MULTIPART_FORM_DATA)
        );

        // then
        perform.andDo(print())
                .andExpect(status().isOk())
                .andExpect(jsonPath("$.code").value(200))
                .andExpect(jsonPath("$.status").value("OK"))
                .andExpect(jsonPath("$.message").value("OK"))
                .andExpect(jsonPath("$.data.id").value(1L))
                .andExpect(jsonPath("$.data.user.id").value(1L))
                .andExpect(jsonPath("$.data.user.username").value("testuser@gmail.com"))
                .andExpect(jsonPath("$.data.user.nickname").value("유저1"))
                .andExpect(jsonPath("$.data.reportType").value("FLIPPED"))
                .andExpect(jsonPath("$.data.parkingLot.id").value(1L))
                .andExpect(jsonPath("$.data.parkingLot.name").value("장덕 공영 주차장"))
                .andExpect(jsonPath("$.data.content").value("로봇이 뒤집혔습니다."))
                .andExpect(jsonPath("$.data.image").value("https://mockurl.com/test-image.jpg"))
                .andExpect(jsonPath("$.data.read").value(false))
                .andExpect(jsonPath("$.data.createdAt").exists());
    }

    @DisplayName("신고 유형이 없다면 예외가 발생한다.")
    @WithMockUser
    @Test
    public void createReportWithMissingReportType() throws Exception {
        // given
        ReportRequest request = ReportRequest.builder()
                .serialNumber("1234ABCD")
                .content("로봇이 뒤집혔습니다.")
                .build();

        MockMultipartFile reportPart = new MockMultipartFile(
                "report",
                "",
                "application/json",
                objectMapper.writeValueAsBytes(request)
        );

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
                        .with(csrf())
                        .header("Authorization", "Bearer token")
                        .contentType(MediaType.MULTIPART_FORM_DATA)
        );

        // then
        perform.andDo(print())
                .andExpect(status().isBadRequest())
                .andExpect(jsonPath("$.code").value(400))
                .andExpect(jsonPath("$.status").value("BAD_REQUEST"))
                .andExpect(jsonPath("$.message").value("신고 유형은 필수입니다."));
    }

    @DisplayName("신고 내용이 없다면 예외가 발생한다.")
    @WithMockUser
    @Test
    public void createReportWithMissingContent() throws Exception {
        // given
        ReportRequest request = ReportRequest.builder()
                .serialNumber("1234ABCD")
                .type(ReportType.FLIPPED)
                .build();

        MockMultipartFile reportPart = new MockMultipartFile(
                "report",
                "",
                "application/json",
                objectMapper.writeValueAsBytes(request)
        );

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
                        .with(csrf())
                        .header("Authorization", "Bearer token")
                        .contentType(MediaType.MULTIPART_FORM_DATA)
        );

        // then
        perform.andDo(print())
                .andExpect(status().isBadRequest())
                .andExpect(jsonPath("$.code").value(400))
                .andExpect(jsonPath("$.status").value("BAD_REQUEST"))
                .andExpect(jsonPath("$.message").value("신고 내용은 필수입니다."));
    }

    @DisplayName("신고 읽기 요청이 성공하면, isRead가 true로 변경된다.")
    @WithMockUser
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

        ReportResponse response = ReportResponse.builder()
                .id(1L)
                .user(userResponse)
                .reportType(ReportType.FLIPPED)
                .parkingLot(parkingLotReportResponse)
                .content("로봇이 뒤집혔습니다.")
                .image("https://mockurl.com/test-image.jpg")
                .isRead(true)  // 읽음 처리
                .createdAt(LocalDateTime.now())
                .build();

        willDoNothing().given(reportService).readReport(any(), anyLong());
        given(reportService.getReport(1L)).willReturn(response);

        // when
        ResultActions perform = mockMvc.perform(
                patch("/api/v1/reports/{reportId}", 1L)
                        .with(csrf())
                        .header("Authorization", "Bearer token")
                        .contentType(MediaType.APPLICATION_JSON)
        );
        // then
        perform.andDo(print())
                .andExpect(status().isOk())
                .andExpect(jsonPath("$.code").value(200))
                .andExpect(jsonPath("$.status").value("OK"))
                .andExpect(jsonPath("$.message").value("OK"))
                .andExpect(jsonPath("$.data.read").value(true));  // isRead가 true인지
    }

    @DisplayName("신고 상세 조회 요청이 성공하면 신고 상세 정보가 정상적으로 반환된다.")
    @WithMockUser
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

        ReportResponse response = ReportResponse.builder()
                .id(1L)
                .user(userResponse)
                .reportType(ReportType.FLIPPED)
                .parkingLot(parkingLotReportResponse)
                .content("로봇이 뒤집혔습니다.")
                .image("https://mockurl.com/test-image.jpg")
                .isRead(true)
                .createdAt(LocalDateTime.now())
                .build();

        given(reportService.getReport(anyLong())).willReturn(response);

        // when
        ResultActions perform = mockMvc.perform(
                get("/api/v1/reports/{reportId}", 1L)
                        .with(csrf())
                        .header("Authorization", "Bearer token")
                        .contentType(MediaType.APPLICATION_JSON)
        );

        // then
        perform.andDo(print())
                .andExpect(status().isOk())
                .andExpect(jsonPath("$.code").value(200))
                .andExpect(jsonPath("$.status").value("OK"))
                .andExpect(jsonPath("$.message").value("OK"))
                .andExpect(jsonPath("$.data.id").value(1L))
                .andExpect(jsonPath("$.data.user.id").value(1L))
                .andExpect(jsonPath("$.data.user.username").value("testuser@gmail.com"))
                .andExpect(jsonPath("$.data.user.nickname").value("유저1"))
                .andExpect(jsonPath("$.data.reportType").value("FLIPPED"))
                .andExpect(jsonPath("$.data.parkingLot.id").value(1L))
                .andExpect(jsonPath("$.data.parkingLot.name").value("장덕 공영 주차장"))
                .andExpect(jsonPath("$.data.content").value("로봇이 뒤집혔습니다."))
                .andExpect(jsonPath("$.data.image").value("https://mockurl.com/test-image.jpg"))
                .andExpect(jsonPath("$.data.read").value(true))
                .andExpect(jsonPath("$.data.createdAt").exists());
    }

    @DisplayName("신고 목록 조회 요청이 성공하면 신고 목록 정보가 정상적으로 반환된다.")
    @WithMockUser
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
                        .with(csrf())
                        .header("Authorization", "Bearer token")
                        .contentType(MediaType.APPLICATION_JSON)
        );

        // then
        perform.andDo(print())
                .andExpect(status().isOk())
                .andExpect(jsonPath("$.code").value(200))
                .andExpect(jsonPath("$.status").value("OK"))
                .andExpect(jsonPath("$.message").value("OK"))
                .andExpect(jsonPath("$.data").isArray())
                .andExpect(jsonPath("$.data[0].id").value(1L))
                .andExpect(jsonPath("$.data[0].user.id").value(1L))
                .andExpect(jsonPath("$.data[0].user.username").value("testuser1@gmail.com"))
                .andExpect(jsonPath("$.data[0].reportType").value("FLIPPED"))
                .andExpect(jsonPath("$.data[0].parkingLot.id").value(1L))
                .andExpect(jsonPath("$.data[0].parkingLot.name").value("장덕 공영 주차장"))
                .andExpect(jsonPath("$.data[0].read").value(false))
                .andExpect(jsonPath("$.data[1].id").value(2L))
                .andExpect(jsonPath("$.data[1].user.id").value(2L))
                .andExpect(jsonPath("$.data[1].user.username").value("testuser2@gmail.com"))
                .andExpect(jsonPath("$.data[1].reportType").value("BROKEN"))
                .andExpect(jsonPath("$.data[1].parkingLot.id").value(1L))
                .andExpect(jsonPath("$.data[1].parkingLot.name").value("장덕 공영 주차장"))
                .andExpect(jsonPath("$.data[1].read").value(true));
    }
}
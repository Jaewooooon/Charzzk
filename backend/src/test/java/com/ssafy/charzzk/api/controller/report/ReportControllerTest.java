//package com.ssafy.charzzk.api.controller.report;
//
//import com.ssafy.charzzk.ControllerTestSupport;
//import com.ssafy.charzzk.api.controller.report.request.ReportRequest;
//import com.ssafy.charzzk.api.service.report.response.ReportResponse;
//import com.ssafy.charzzk.domain.report.ReportType;
//import org.junit.jupiter.api.DisplayName;
//import org.junit.jupiter.api.Test;
//import org.springframework.http.MediaType;
//import org.springframework.security.test.context.support.WithMockUser;
//import org.springframework.test.web.servlet.ResultActions;
//
//import java.time.LocalDateTime;
//
//import static org.junit.jupiter.api.Assertions.*;
//import static org.mockito.BDDMockito.given;
//import static org.mockito.ArgumentMatchers.any;
//import static org.springframework.security.test.web.servlet.request.SecurityMockMvcRequestPostProcessors.csrf;
//import static org.springframework.test.web.servlet.request.MockMvcRequestBuilders.*;
//import static org.springframework.test.web.servlet.result.MockMvcResultHandlers.print;
//import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.jsonPath;
//import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.status;
//
//
//class ReportControllerTest extends ControllerTestSupport {
//
//    @DisplayName("신고하기 요청이 성공하면 신고가 정상적으로 생성된다.")
//    @WithMockUser
//    @Test
//    public void createReport() throws Exception {
//        // given
//        ReportRequest request = ReportRequest.builder()
//                .serialNumber("1234ABCD")
//                .type(ReportType.FLIPPED)
//                .content("로봇이 뒤집혔습니다.")
//                .image("test-image-url")
//                .build();
//
//        ReportResponse response = ReportResponse.builder()
//                .id(1L)
//                .content("로봇이 뒤집혔습니다.")
//                .image("test-image-url")
//                .reportType(ReportType.FLIPPED)
//                .createdAt(LocalDateTime.now())
//                .build();
//
//        given(reportService.createReport(any(), any())).willReturn(1L);
//        given(reportService.getReport(1L)).willReturn(response);
//
//        // when
//        ResultActions perform = mockMvc.perform(
//                post("/api/v1/reports")
//                        .with(csrf())
//                        .content(objectMapper.writeValueAsString(request))
//                        .contentType(MediaType.APPLICATION_JSON)
//        );
//
//        // then
//        perform.andDo(print())
//                .andExpect(status().isOk())
//                .andExpect(jsonPath("$.code").value(200))
//                .andExpect(jsonPath("$.status").value("OK"))
//                .andExpect(jsonPath("$.message").value("OK"))
//                .andExpect(jsonPath("$.data.content").value("로봇이 뒤집혔습니다."))
//                .andExpect(jsonPath("$.data.image").value("test-image-url"));
//    }
//
//    @DisplayName("신고 유형이 없다면 예외가 발생한다.")
//    @WithMockUser
//    @Test
//    public void createReportWithMissingReportType() throws Exception {
//        // given
//        ReportRequest request = ReportRequest.builder()
//                .serialNumber("1234ABCD")
//                .content("로봇이 뒤집혔습니다.")
//                .image("test-image-url")
//                .build();
//
//        // when
//        ResultActions perform = mockMvc.perform(
//                post("/api/v1/reports")
//                        .with(csrf())
//                        .content(objectMapper.writeValueAsString(request))
//                        .contentType(MediaType.APPLICATION_JSON)
//        );
//
//        // then
//        perform.andDo(print())
//                .andExpect(status().isBadRequest())
//                .andExpect(jsonPath("$.code").value(400))
//                .andExpect(jsonPath("$.status").value("BAD_REQUEST"))
//                .andExpect(jsonPath("$.message").value("신고 유형은 필수입니다."));
//    }
//
//    @DisplayName("신고 내용이 없다면 예외가 발생한다.")
//    @WithMockUser
//    @Test
//    public void createReportWithMissingContent() throws Exception {
//        // given
//        ReportRequest request = ReportRequest.builder()
//                .serialNumber("1234ABCD")
//                .type(ReportType.FLIPPED)
//                .image("test-image-url")
//                .build();
//
//        // when
//        ResultActions perform = mockMvc.perform(
//                post("/api/v1/reports")
//                        .with(csrf())
//                        .content(objectMapper.writeValueAsString(request))
//                        .contentType(MediaType.APPLICATION_JSON)
//        );
//
//        // then
//        perform.andDo(print())
//                .andExpect(status().isBadRequest())
//                .andExpect(jsonPath("$.code").value(400))
//                .andExpect(jsonPath("$.status").value("BAD_REQUEST"))
//                .andExpect(jsonPath("$.message").value("신고 내용은 필수입니다."));
//    }
//
//}
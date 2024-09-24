package com.ssafy.charzzk.api.service.report;

import com.ssafy.charzzk.IntegrationTestSupport;
import com.ssafy.charzzk.api.service.report.request.ReportServiceRequest;
import com.ssafy.charzzk.domain.charger.Charger;
import com.ssafy.charzzk.domain.charger.ChargerRepository;
import com.ssafy.charzzk.domain.parkinglot.ParkingLot;
import com.ssafy.charzzk.domain.report.ReportRepository;
import com.ssafy.charzzk.domain.report.ReportType;
import com.ssafy.charzzk.domain.user.User;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.test.context.ActiveProfiles;
import org.springframework.transaction.annotation.Transactional;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.api.Assertions.*;

import static org.mockito.BDDMockito.given;
import static org.mockito.ArgumentMatchers.any;
import static org.springframework.security.test.web.servlet.request.SecurityMockMvcRequestPostProcessors.csrf;
import static org.springframework.test.web.servlet.request.MockMvcRequestBuilders.*;
import static org.springframework.test.web.servlet.result.MockMvcResultHandlers.print;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.jsonPath;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.status;

@Transactional
@ActiveProfiles("test")
class ReportServiceTest extends IntegrationTestSupport {

    @Autowired
    private ReportService reportService;

    @Autowired
    private ReportRepository reportRepository;

    @Autowired
    private ChargerRepository chargerRepository;

//    private User user;
//    private ParkingLot parkingLot;
//    private Charger charger;
//
//    // 테스트 전 사용자, 주차장, 충전 로봇 세팅
//    @BeforeEach
//    public void setUp() {
//        user = User.builder()
//                .username("testuser@gmail.com")
//                .nickname("테스트유저")
//                .build();
//
//        parkingLot = ParkingLot.builder()
//                .name("테스트 주차장")
//                .build();
//
//        charger = Charger.builder()
//                .serialNumber("1234ABCD")
//                .parkingLot(parkingLot)
//                .build();
//
//        chargerRepository.save(charger);
//    }

//    @DisplayName("신고를 등록하면 정상적으로 등록된다.")
//    @Test
//    public void createReport() throws Exception {
//        // given
//        ReportServiceRequest request = ReportServiceRequest.builder()
//                .serialNumber("1234ABCD")
//                .type(ReportType.FLIPPED)
//                .content("로봇이 뒤집혔습니다.")
//                .image("test-image-url")
//                .build();
//
//        // when
//        Long reportId = reportService.createReport(user, request);
//
//        // then
//        assertThat(reportId).isNotNull();
//
//    }

  
}
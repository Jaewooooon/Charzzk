package com.ssafy.charzzk.api.service.report;

import com.ssafy.charzzk.IntegrationTestSupport;
import com.ssafy.charzzk.api.service.parkinglot.ParkingLotService;
import com.ssafy.charzzk.api.service.report.request.ReportServiceRequest;
import com.ssafy.charzzk.domain.charger.Charger;
import com.ssafy.charzzk.domain.charger.ChargerRepository;
import com.ssafy.charzzk.domain.parkinglot.Location;
import com.ssafy.charzzk.domain.parkinglot.ParkingLot;
import com.ssafy.charzzk.domain.parkinglot.ParkingLotRepository;
import com.ssafy.charzzk.domain.report.Report;
import com.ssafy.charzzk.domain.report.ReportRepository;
import com.ssafy.charzzk.domain.report.ReportType;
import com.ssafy.charzzk.domain.user.User;
import com.ssafy.charzzk.domain.user.UserRepository;
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

    @Autowired
    private ParkingLotRepository parkingLotRepository;

    @Autowired
    private UserRepository userRepository;

    @DisplayName("신고를 등록하면 정상적으로 등록된다.")
    @Test
    public void createReport() throws Exception {
        // given

        User user = User.builder()
                .username("testuser@gmail.com")
                .nickname("테스트유저")
                .build();

        userRepository.save(user);

        Location location = Location.builder()
                .latitude(37.5665)
                .longitude(126.9780)
                .build();

        ParkingLot parkingLot = ParkingLot.builder()
                .name("테스트 주차장")
                .location(location)
                .build();

        parkingLotRepository.save(parkingLot);

        Charger charger = Charger.builder()
                .serialNumber("1234ABCD")
                .parkingLot(parkingLot)
                .build();

        chargerRepository.save(charger);

        ReportServiceRequest request = ReportServiceRequest.builder()
                .serialNumber("1234ABCD")
                .type(ReportType.FLIPPED)
                .content("로봇이 뒤집혔습니다.")
                .image("test-image-url")
                .build();

        // when
        Long reportId = reportService.createReport(user, request);
        Report savedReport = reportRepository.findById(reportId).orElseThrow();

        // then
        assertThat(reportId).isNotNull();
        assertThat(savedReport.getContent()).isEqualTo("로봇이 뒤집혔습니다.");
        assertThat(savedReport.getType()).isEqualTo(ReportType.FLIPPED);
        assertThat(savedReport.getUser().getUsername()).isEqualTo("testuser@gmail.com");
        assertThat(savedReport.getParkingLot().getName()).isEqualTo("테스트 주차장");
        assertThat(savedReport.getParkingLot().getLocation().getLatitude()).isEqualTo(37.5665);
        assertThat(savedReport.getParkingLot().getLocation().getLongitude()).isEqualTo(126.9780);
    }


}
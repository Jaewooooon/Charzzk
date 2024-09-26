package com.ssafy.charzzk.api.service.report;

import com.ssafy.charzzk.IntegrationTestSupport;
import com.ssafy.charzzk.api.service.report.request.ReportServiceRequest;
import com.ssafy.charzzk.api.service.report.response.ReportListResponse;
import com.ssafy.charzzk.api.service.report.response.ReportResponse;
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
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.mock.web.MockMultipartFile;
import org.springframework.test.context.ActiveProfiles;
import org.springframework.transaction.annotation.Transactional;
import org.springframework.web.multipart.MultipartFile;

import java.util.List;

import static com.ssafy.charzzk.domain.report.ReportType.*;
import static org.assertj.core.api.Assertions.assertThat;

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

    @DisplayName("이미지와 함께 신고를 등록하면 이미지가 정상적으로 업로드되고 신고가 등록된다.")
    @Test
    public void createReportWithImage() {
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

        MultipartFile mockImage = new MockMultipartFile("image", "test-image.jpg", "image/jpeg", "image content".getBytes());

        ReportServiceRequest request = ReportServiceRequest.builder()
                .serialNumber("1234ABCD")
                .type(ReportType.FLIPPED)
                .content("로봇이 뒤집혔습니다.")
                .build();

        // when
        Long reportId = reportService.createReport(user, request, mockImage);
        Report savedReport = reportRepository.findByIdWithUserAndParkingLot(reportId).orElseThrow();

        // then
        assertThat(reportId).isNotNull();
        assertThat(savedReport.getContent()).isEqualTo("로봇이 뒤집혔습니다.");
        assertThat(savedReport.getType()).isEqualTo(ReportType.FLIPPED);
        assertThat(savedReport.getUser().getUsername()).isEqualTo("testuser@gmail.com");
        assertThat(savedReport.getParkingLot().getName()).isEqualTo("테스트 주차장");
        assertThat(savedReport.getImage()).contains("test-image.jpg");
    }

    @DisplayName("이미지 없이 신고를 등록하면 이미지 필드는 null로 저장된다.")
    @Test
    public void createReportWithoutImage() {
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
                .build();

        // when
        Long reportId = reportService.createReport(user, request, null);
        Report savedReport = reportRepository.findByIdWithUserAndParkingLot(reportId).orElseThrow();

        // then
        assertThat(reportId).isNotNull();
        assertThat(savedReport.getContent()).isEqualTo("로봇이 뒤집혔습니다.");
        assertThat(savedReport.getType()).isEqualTo(ReportType.FLIPPED);
        assertThat(savedReport.getUser().getUsername()).isEqualTo("testuser@gmail.com");
        assertThat(savedReport.getParkingLot().getName()).isEqualTo("테스트 주차장");
        assertThat(savedReport.getImage()).isNull();
    }


    @DisplayName("ID로 신고를 조회하면 정상적으로 조회된다.")
    @Test
    public void getReport() {
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

        Report report = Report.create(user, parkingLot, ReportType.FLIPPED, "로봇이 뒤집혔습니다.", "test-image-url");
        reportRepository.save(report);

        // when
        ReportResponse foundReport = reportService.getReport(report.getId());

        // then
        assertThat(foundReport).isNotNull();
        assertThat(foundReport.getContent()).isEqualTo("로봇이 뒤집혔습니다.");
        assertThat(foundReport.getReportType()).isEqualTo(ReportType.FLIPPED);
        assertThat(foundReport.getImage()).isEqualTo("test-image-url");
        assertThat(foundReport.getUser().getUsername()).isEqualTo("testuser@gmail.com");
        assertThat(foundReport.getParkingLot().getName()).isEqualTo("테스트 주차장");
    }

    @DisplayName("신고를 읽음 처리하면 isRead가 true로 변경된다.")
    @Test
    public void readReport() {
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

        Report report = Report.create(user, parkingLot, ReportType.FLIPPED, "로봇이 뒤집혔습니다.", null);
        reportRepository.save(report);

        // when
        reportService.readReport(user, report.getId());

        // then
        Report updatedReport = reportRepository.findByIdWithUserAndParkingLot(report.getId()).orElseThrow();
        assertThat(updatedReport.isRead()).isTrue();
    }

    @DisplayName("이미 읽힌 신고를 다시 읽음 처리하면 상태가 변하지 않는다.")
    @Test
    public void readReport_alreadyRead() {
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

        Report report = Report.create(user, parkingLot, ReportType.FLIPPED, "로봇이 뒤집혔습니다.", null);
        report.readReport();
        reportRepository.save(report);

        // when & then
        assertThat(report.isRead()).isTrue();

        reportService.readReport(user, report.getId());

        Report updatedReport = reportRepository.findByIdWithUserAndParkingLot(report.getId()).orElseThrow();
        assertThat(updatedReport.isRead()).isTrue();
    }

    @DisplayName("모든 신고 리스트가 정상적으로 조회된다.")
    @Test
    public void getReportList() {
        // given
        User user1 = User.builder()
                .username("testuser1@gmail.com")
                .nickname("테스트유저1")
                .build();
        User user2 = User.builder()
                .username("testuser2@gmail.com")
                .nickname("테스트유저2")
                .build();
        User user3 = User.builder()
                .username("testuser3@gmail.com")
                .nickname("테스트유저3")
                .build();
        userRepository.saveAll(List.of(user1, user2, user3));

        Location location1 = Location.builder()
                .latitude(37.5665)
                .longitude(126.9780)
                .build();

        Location location2 = Location.builder()
                .latitude(37.4165)
                .longitude(126.9880)
                .build();

        ParkingLot parkingLot1 = ParkingLot.builder()
                .name("테스트 주차장1")
                .location(location1)
                .build();
        ParkingLot parkingLot2 = ParkingLot.builder()
                .name("테스트 주차장2")
                .location(location2)
                .build();
        parkingLotRepository.saveAll(List.of(parkingLot1, parkingLot2));

        Report report1 = Report.builder()
                .user(user1)
                .parkingLot(parkingLot1)
                .type(ReportType.FLIPPED)
                .content("로봇이 뒤집혔습니다.")
                .image("test-image-url1")
                .isRead(false)
                .build();

        Report report2 = Report.builder()
                .user(user2)
                .parkingLot(parkingLot2)
                .type(BROKEN)
                .content("로봇이 고장났습니다.")
                .image("test-image-url2")
                .isRead(true)
                .build();

        Report report3 = Report.builder()
                .user(user3)
                .parkingLot(parkingLot1)
                .type(ETC)
                .content("로봇이 오작동합니다.")
                .image("test-image-url3")
                .isRead(false)
                .build();

        reportRepository.saveAll(List.of(report1, report2, report3));

        // when
        List<ReportListResponse> reportList = reportService.getReportList();

        // then
        assertThat(reportList).hasSize(3)
                .extracting("reportType")
                .containsExactly(FLIPPED, BROKEN, ETC);

        assertThat(reportList).extracting("user.username")
                .containsExactly("testuser1@gmail.com", "testuser2@gmail.com", "testuser3@gmail.com");

        assertThat(reportList).extracting("parkingLot.name")
                .containsExactly("테스트 주차장1", "테스트 주차장2", "테스트 주차장1");
    }

}
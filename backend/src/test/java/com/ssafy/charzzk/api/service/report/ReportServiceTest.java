package com.ssafy.charzzk.api.service.report;

import com.ssafy.charzzk.IntegrationTestSupport;
import com.ssafy.charzzk.api.service.report.request.ReportServiceRequest;
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
    public void createReportWithImage() throws Exception {
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
        Report savedReport = reportRepository.findById(reportId).orElseThrow();

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
    public void createReportWithoutImage() throws Exception {
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
        Report savedReport = reportRepository.findById(reportId).orElseThrow();

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
    public void getReport() throws Exception {
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
    public void readReport() throws Exception {
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
        Report updatedReport = reportRepository.findById(report.getId()).orElseThrow();
        assertThat(updatedReport.isRead()).isTrue();
    }

    @DisplayName("이미 읽힌 신고를 다시 읽음 처리하면 상태가 변하지 않는다.")
    @Test
    public void readReport_alreadyRead() throws Exception {
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

        Report updatedReport = reportRepository.findById(report.getId()).orElseThrow();
        assertThat(updatedReport.isRead()).isTrue();
    }

}
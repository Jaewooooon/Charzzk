package com.ssafy.charzzk.domain.report;

import com.ssafy.charzzk.IntegrationTestSupport;
import com.ssafy.charzzk.domain.parkinglot.Location;
import com.ssafy.charzzk.domain.parkinglot.ParkingLot;
import com.ssafy.charzzk.domain.parkinglot.ParkingLotRepository;
import com.ssafy.charzzk.domain.user.User;
import com.ssafy.charzzk.domain.user.UserRepository;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.test.context.ActiveProfiles;
import org.springframework.transaction.annotation.Transactional;

import java.util.Optional;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.api.Assertions.*;

@Transactional
@ActiveProfiles("test")
class ReportRepositoryTest extends IntegrationTestSupport {
    @Autowired
    private ReportRepository reportRepository;

    @Autowired
    private UserRepository userRepository;

    @Autowired
    private ParkingLotRepository parkingLotRepository;

    @DisplayName("ID로 조회하면 신고가 정상적으로 조회된다.")
    @Test
    void findById() {
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

        Report report = Report.builder()
                .user(user)
                .parkingLot(parkingLot)
                .type(ReportType.FLIPPED)
                .content("로봇이 뒤집혔습니다.")
                .image("test-image-url")
                .isRead(false)
                .build();

        reportRepository.save(report);

        // when
        Optional<Report> foundReport = reportRepository.findById(report.getId());

        // then
        assertThat(foundReport).isPresent();
        assertThat(foundReport.get().getContent()).isEqualTo("로봇이 뒤집혔습니다.");
        assertThat(foundReport.get().getUser().getUsername()).isEqualTo("testuser@gmail.com");
        assertThat(foundReport.get().getParkingLot().getName()).isEqualTo("테스트 주차장");
        assertThat(foundReport.get().getParkingLot().getLocation().getLatitude()).isEqualTo(37.5665);
        assertThat(foundReport.get().getParkingLot().getLocation().getLongitude()).isEqualTo(126.9780);
    }

    @DisplayName("존재하지 않는 ID로 신고 조회 시 빈 Optional을 반환한다.")
    @Test
    void findByIdNotFound() {
        // when
        Optional<Report> foundReport = reportRepository.findById(999L);

        // then
        assertThat(foundReport).isNotPresent();
    }
}
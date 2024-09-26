package com.ssafy.charzzk.domain.report;

import com.ssafy.charzzk.IntegrationTestSupport;
import com.ssafy.charzzk.domain.parkinglot.Location;
import com.ssafy.charzzk.domain.parkinglot.ParkingLot;
import com.ssafy.charzzk.domain.parkinglot.ParkingLotRepository;
import com.ssafy.charzzk.domain.user.User;
import com.ssafy.charzzk.domain.user.UserRepository;
import org.assertj.core.groups.Tuple;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.test.context.ActiveProfiles;
import org.springframework.transaction.annotation.Transactional;

import java.util.List;
import java.util.Optional;

import static com.ssafy.charzzk.domain.report.ReportType.*;
import static org.assertj.core.api.Assertions.assertThat;

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
    void findByIdWithUserAndParkingLot() {
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
                .type(FLIPPED)
                .content("로봇이 뒤집혔습니다.")
                .image("test-image-url")
                .isRead(false)
                .build();

        reportRepository.save(report);

        // when
        Optional<Report> foundReport = reportRepository.findByIdWithUserAndParkingLot(report.getId());

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
    void findByIdWithUserAndParkingLotNotFound() {
        // when
        Optional<Report> foundReport = reportRepository.findByIdWithUserAndParkingLot(999L);

        // then
        assertThat(foundReport).isNotPresent();
    }

    @DisplayName("모든 신고 정보가 유저 및 주차장 정보와 함께 정상적으로 조회된다.")
    @Test
    void findAllWithUserAndParkingLot() {
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

        Location location = Location.builder()
                .latitude(37.5665)
                .longitude(126.9780)
                .build();

        ParkingLot parkingLot = ParkingLot.builder()
                .name("테스트 주차장")
                .location(location)
                .build();
        parkingLotRepository.save(parkingLot);

        Report report1 = Report.builder()
                .user(user1)
                .parkingLot(parkingLot)
                .type(FLIPPED)
                .content("로봇이 뒤집혔습니다.")
                .image("test-image-url1")
                .isRead(false)
                .build();

        Report report2 = Report.builder()
                .user(user2)
                .parkingLot(parkingLot)
                .type(BROKEN)
                .content("로봇이 고장났습니다.")
                .image("test-image-url2")
                .isRead(true)
                .build();

        Report report3 = Report.builder()
                .user(user3)
                .parkingLot(parkingLot)
                .type(ETC)
                .content("로봇이 멈췄습니다.")
                .image("test-image-url3")
                .isRead(false)
                .build();

        reportRepository.saveAll(List.of(report1, report2, report3));

        // when
        List<Report> reportList = reportRepository.findAllWithUserAndParkingLot();

        // then
        assertThat(reportList).hasSize(3)
                .extracting("type", "user.username", "parkingLot.name")
                .containsExactlyInAnyOrder(
                        Tuple.tuple(FLIPPED, "testuser1@gmail.com", "테스트 주차장"),
                        Tuple.tuple(BROKEN, "testuser2@gmail.com", "테스트 주차장"),
                        Tuple.tuple(ETC, "testuser3@gmail.com", "테스트 주차장")
                );
    }
}
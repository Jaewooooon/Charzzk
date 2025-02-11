package com.ssafy.charzzk;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.ssafy.charzzk.api.controller.car.CarController;
import com.ssafy.charzzk.api.controller.charginglog.ChargingLogController;
import com.ssafy.charzzk.api.controller.parkinglot.ParkingLotController;
import com.ssafy.charzzk.api.controller.report.ReportController;
import com.ssafy.charzzk.api.controller.reservation.ReservationController;
import com.ssafy.charzzk.api.controller.user.UserController;
import com.ssafy.charzzk.api.service.auth.CustomUserService;
import com.ssafy.charzzk.api.service.auth.JWTService;
import com.ssafy.charzzk.api.service.car.CarService;
import com.ssafy.charzzk.api.service.charginglog.ChargingLogService;
import com.ssafy.charzzk.api.service.parkinglot.ParkingLotService;
import com.ssafy.charzzk.api.service.report.ReportService;
import com.ssafy.charzzk.api.service.report.S3ImageService;
import com.ssafy.charzzk.api.service.reservation.ReservationService;
import com.ssafy.charzzk.api.service.user.UserService;
import com.ssafy.charzzk.core.configuration.SecurityConfig;
import com.ssafy.charzzk.core.filter.JWTFilter;
import com.ssafy.charzzk.domain.user.User;
import com.ssafy.charzzk.domain.user.UserRepository;
import org.junit.jupiter.api.BeforeEach;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.test.autoconfigure.web.servlet.WebMvcTest;
import org.springframework.boot.test.mock.mockito.MockBean;
import org.springframework.context.annotation.ComponentScan;
import org.springframework.context.annotation.FilterType;
import org.springframework.context.annotation.Import;
import org.springframework.test.web.servlet.MockMvc;

import java.util.Optional;

import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.when;

@WebMvcTest(
        controllers = {
                UserController.class,
                CarController.class,
                ParkingLotController.class,
                ReportController.class,
                ParkingLotController.class,
                ReservationController.class,
                ChargingLogController.class,
        },
        excludeFilters = {
                @ComponentScan.Filter(type = FilterType.ASSIGNABLE_TYPE, classes = {SecurityConfig.class}),
                @ComponentScan.Filter(type = FilterType.ASSIGNABLE_TYPE, classes = {JWTFilter.class})
        }
)
@Import(CustomUserService.class)
public abstract class ControllerTestSupport {

    @Autowired
    protected MockMvc mockMvc;

    @Autowired
    protected ObjectMapper objectMapper;

    @MockBean
    protected UserService userService;

    @MockBean
    protected CarService carService;

    @MockBean
    protected ParkingLotService parkingLotService;

    @MockBean
    protected ReportService reportService;

    @MockBean
    protected S3ImageService s3ImageService;

    @MockBean
    protected ReservationService reservationService;

    @MockBean
    protected JWTService jwtService;

    @MockBean
    protected UserRepository userRepository;

    @MockBean
    protected ChargingLogService chargingLogService;

    @BeforeEach
    public void setUp() {
        User user = User.create("test-user", "test-user");
        when(userRepository.findByUsername(any())).thenReturn(Optional.of(user));
    }
}

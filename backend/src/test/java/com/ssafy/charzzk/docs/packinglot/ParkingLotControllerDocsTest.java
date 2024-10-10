package com.ssafy.charzzk.docs.packinglot;

import com.epages.restdocs.apispec.ResourceSnippetParameters;
import com.ssafy.charzzk.api.controller.parkinglot.ParkingLotController;
import com.ssafy.charzzk.api.controller.parkinglot.request.ParkingLotListRequest;
import com.ssafy.charzzk.api.service.charger.response.ChargerResponse;
import com.ssafy.charzzk.api.service.parkinglot.ParkingLotService;
import com.ssafy.charzzk.api.service.parkinglot.response.ParkingLotListResponse;
import com.ssafy.charzzk.api.service.parkinglot.response.ParkingLotResponse;
import com.ssafy.charzzk.api.service.parkinglot.response.ParkingSpotListResponse;
import com.ssafy.charzzk.api.service.reservation.response.ReservationSimpleResponse;
import com.ssafy.charzzk.docs.RestDocsSupport;
import com.ssafy.charzzk.domain.car.Car;
import com.ssafy.charzzk.domain.car.CarType;
import com.ssafy.charzzk.domain.charger.Charger;
import com.ssafy.charzzk.domain.charger.ChargerStatus;
import com.ssafy.charzzk.domain.parkinglot.Location;
import com.ssafy.charzzk.domain.parkinglot.ParkingLot;
import com.ssafy.charzzk.domain.parkinglot.ParkingSpot;
import com.ssafy.charzzk.domain.reservation.Reservation;
import com.ssafy.charzzk.domain.reservation.ReservationStatus;
import com.ssafy.charzzk.domain.user.User;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.springframework.http.MediaType;
import org.springframework.restdocs.payload.JsonFieldType;
import org.springframework.test.web.servlet.ResultActions;

import java.time.LocalDateTime;
import java.util.List;
import java.util.stream.Stream;

import static com.epages.restdocs.apispec.MockMvcRestDocumentationWrapper.document;
import static com.epages.restdocs.apispec.ResourceDocumentation.resource;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.BDDMockito.given;
import static org.mockito.Mockito.mock;
import static org.springframework.restdocs.mockmvc.RestDocumentationRequestBuilders.get;
import static org.springframework.restdocs.operation.preprocess.Preprocessors.preprocessResponse;
import static org.springframework.restdocs.operation.preprocess.Preprocessors.prettyPrint;
import static org.springframework.restdocs.payload.PayloadDocumentation.fieldWithPath;
import static org.springframework.restdocs.request.RequestDocumentation.parameterWithName;
import static org.springframework.security.test.web.servlet.request.SecurityMockMvcRequestPostProcessors.csrf;
import static org.springframework.test.web.servlet.result.MockMvcResultHandlers.print;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.status;

class ParkingLotControllerDocsTest extends RestDocsSupport {

    private final ParkingLotService parkingLotService = mock(ParkingLotService.class);

    @Override
    protected Object initController() {
        return new ParkingLotController(parkingLotService);
    }

    @DisplayName("주변 주차장 리스트 현재 거리와 가까운순 오름차순으로 조회한다.")
    @Test
    void getParkingLotList() throws Exception {
        // given
        ParkingLotListRequest request = ParkingLotListRequest.builder()
                .latitude(37.123456)
                .longitude(127.123456)
                .build();

        Location location = Location.builder()
                .latitude(0.0)
                .longitude(0.0)
                .build();

        List<ParkingLotListResponse> response = List.of(
                ParkingLotListResponse.builder()
                        .id(1L)
                        .name("주차장1")
                        .location(location)
                        .image("주차장 이미지")
                        .distance(100.0)
                        .build()
        );

        given(parkingLotService.getParkingLotList(anyDouble(), anyDouble(), anyString(), anyString(), any())).willReturn(response);

        // when
        ResultActions perform = mockMvc.perform(
                get("/api/v1/parking-lot")
                        .param("q", "검색어")
                        .param("latitude", "37.123456")
                        .param("longitude", "127.123456")
                        .content(objectMapper.writeValueAsString(request))
                        .contentType(MediaType.APPLICATION_JSON)
        );

        // then
        perform
                .andDo(print())
                .andExpect(status().isOk())
                .andDo(document("parking-lot-list-get",
                        preprocessResponse(prettyPrint()),
                        resource(ResourceSnippetParameters.builder()
                                .tag("ParkingLot")
                                .summary("주차장 목록 조회")
                                .requestFields(
                                        fieldWithPath("latitude").type(JsonFieldType.NUMBER)
                                                .description("위도"),
                                        fieldWithPath("longitude").type(JsonFieldType.NUMBER)
                                                .description("경도")
                                )
                                .queryParameters(
                                        parameterWithName("q").optional().description("검색어 (옵션)"),
                                        parameterWithName("sort").optional().description("정렬 조건"),
                                        parameterWithName("latitude").description("위도"),
                                        parameterWithName("longitude").description("경도")
                                )
                                .responseFields(
                                        fieldWithPath("code").type(JsonFieldType.NUMBER)
                                                .description("코드"),
                                        fieldWithPath("status").type(JsonFieldType.STRING)
                                                .description("상태"),
                                        fieldWithPath("message").type(JsonFieldType.STRING)
                                                .description("메시지"),
                                        fieldWithPath("data[].id").type(JsonFieldType.NUMBER)
                                                .description("주차장 아이디"),
                                        fieldWithPath("data[].name").type(JsonFieldType.STRING)
                                                .description("주차장 이름"),
                                        fieldWithPath("data[].location").type(JsonFieldType.OBJECT)
                                                .description("좌표"),
                                        fieldWithPath("data[].location.latitude").type(JsonFieldType.NUMBER)
                                                .description("위도"),
                                        fieldWithPath("data[].location.longitude").type(JsonFieldType.NUMBER)
                                                .description("경도"),
                                        fieldWithPath("data[].image").type(JsonFieldType.STRING)
                                                .description("주차장 이미지"),
                                        fieldWithPath("data[].distance").type(JsonFieldType.NUMBER)
                                                .description("주차장까지 거리"),
                                        fieldWithPath("data[].waitingTime").type(JsonFieldType.NUMBER)
                                                .description("예상 대기 시간"))
                                .build()
                        )
                ));

    }

    @DisplayName("주차장 상세정보를 조회한다.")
    @Test
    void getParkingLot() throws Exception {
        // given
        List<ParkingSpotListResponse> parkingSpotListResponseList = List.of(
                ParkingSpotListResponse.builder()
                        .id(1L)
                        .name("A11")
                        .build(),
                ParkingSpotListResponse.builder()
                        .id(2L)
                        .name("A12")
                        .build()
        );

        ParkingLotResponse response = ParkingLotResponse.builder()
                .id(1L)
                .name("주차장1")
                .parkingMapImage("주차장1 이미지")
                .parkingSpots(parkingSpotListResponseList)
                .build();

        given(parkingLotService.getParkingLot(any())).willReturn(response);

        // when
        ResultActions perform = mockMvc.perform(
                get("/api/v1/parking-lot/{parkingLotId}", 1L)
        );

        // then
        perform
                .andDo(print())
                .andExpect(status().isOk())
                .andDo(document("parking-lot-get",
                        preprocessResponse(prettyPrint()),
                        resource(ResourceSnippetParameters.builder()
                                .tag("ParkingLot")
                                .summary("주차장 상세 조회")
                                .responseFields(
                                        fieldWithPath("code").type(JsonFieldType.NUMBER)
                                                .description("코드"),
                                        fieldWithPath("status").type(JsonFieldType.STRING)
                                                .description("상태"),
                                        fieldWithPath("message").type(JsonFieldType.STRING)
                                                .description("메시지"),
                                        fieldWithPath("data.id").type(JsonFieldType.NUMBER)
                                                .description("주차장 아이디"),
                                        fieldWithPath("data.name").type(JsonFieldType.STRING)
                                                .description("주차장 이름"),
                                        fieldWithPath("data.parkingMapImage").type(JsonFieldType.STRING)
                                                .description("주차칸 이미지"),
                                        fieldWithPath("data.parkingSpots[]").type(JsonFieldType.ARRAY)
                                                .description("주차칸 이미지"),
                                        fieldWithPath("data.parkingSpots[].id").type(JsonFieldType.NUMBER)
                                                .description("주차칸 아이디"),
                                        fieldWithPath("data.parkingSpots[].name").type(JsonFieldType.STRING)
                                                .description("주차칸 이름")
                                )
                                .build())));

    }

    @DisplayName("주차장 ID로 충전 로봇 목록을 조회한다.")
    @Test
    void getChargerList() throws Exception {
        // given
        User user = User.builder()
                .username("user@google.com")
                .nickname("user")
                .build();

        CarType carType = CarType.builder()
                .name("테스트 차종")
                .build();

        Car car = Car.builder()
                .user(user)
                .carType(carType)
                .number("12다1234")
                .build();

        Location location = Location.builder()
                .latitude(37.123456)
                .longitude(127.123456)
                .build();

        ParkingLot parkingLot = ParkingLot.builder()
                .name("테스트 주차장")
                .location(location)
                .build();

        ParkingSpot parkingSpot = ParkingSpot.builder()
                .parkingLot(parkingLot)
                .name("1")
                .location(location)
                .build();

        Charger charger = Charger.builder()
                .parkingLot(parkingLot)
                .serialNumber("1")
                .status(ChargerStatus.WAITING)
                .lastReservedTime(LocalDateTime.of(2024, 1, 1, 2, 0))
                .build();

        parkingLot.getChargers().add(charger);

        Reservation reservation1 = Reservation.builder()
                .id(2L)
                .car(car)
                .parkingSpot(parkingSpot)
                .charger(charger)
                .startTime(LocalDateTime.of(2024, 1, 1, 0, 0))
                .endTime(LocalDateTime.of(2024, 1, 1, 1, 0))
                .status(ReservationStatus.PENDING)
                .build();

        Reservation reservation2 = Reservation.builder()
                .id(2L)
                .car(car)
                .parkingSpot(parkingSpot)
                .charger(charger)
                .startTime(LocalDateTime.of(2024, 2, 1, 0, 0))
                .endTime(LocalDateTime.of(2024, 2, 1, 1, 0))
                .status(ReservationStatus.PENDING)
                .build();

        ChargerResponse chargerResponse1 = ChargerResponse.builder()
                .id(1L)
                .serialNumber("1234A")
                .battery(80)
                .reservations(
                        Stream.of(reservation1, reservation2)
                                .map(ReservationSimpleResponse::from)
                                .toList()
                )
                .status(ChargerStatus.WAITING)
                .build();

        ChargerResponse chargerResponse2 = ChargerResponse.builder()
                .id(2L)
                .serialNumber("1234B")
                .battery(90)
                .reservations(
                        Stream.of(reservation1, reservation2)
                                .map(ReservationSimpleResponse::from)
                                .toList()
                )
                .status(ChargerStatus.CHARGER_CHARGING)
                .build();


        List<ChargerResponse> chargerList = List.of(chargerResponse1, chargerResponse2);

        given(parkingLotService.getChargerList(any(Long.class))).willReturn(chargerList);

        // when
        ResultActions perform = mockMvc.perform(
                get("/api/v1/parking-lot/{parkingLotId}/chargers", 1L)
                        .with(csrf())
                        .contentType(MediaType.APPLICATION_JSON)
        );

        // then
        perform
                .andDo(print())
                .andExpect(status().isOk())
                .andDo(document("parking-lot-chargers-get",
                        preprocessResponse(prettyPrint()),
                        resource(ResourceSnippetParameters.builder()
                                .tag("ParkingLot")
                                .summary("주차장 충전 로봇 목록 조회")
                                .description("주차장 ID로 해당 주차장에 속한 충전 로봇 목록을 조회한다.")
                                .pathParameters(
                                        parameterWithName("parkingLotId").description("주차장 아이디")
                                )
                                .responseFields(
                                        fieldWithPath("code").type(JsonFieldType.NUMBER)
                                                .description("코드"),
                                        fieldWithPath("status").type(JsonFieldType.STRING)
                                                .description("상태"),
                                        fieldWithPath("message").type(JsonFieldType.STRING)
                                                .description("메시지"),
                                        fieldWithPath("data[].chargerId").type(JsonFieldType.NUMBER)
                                                .description("충전 로봇 아이디"),
                                        fieldWithPath("data[].serialNumber").type(JsonFieldType.STRING)
                                                .description("충전 로봇 시리얼 번호"),
                                        fieldWithPath("data[].battery").type(JsonFieldType.NUMBER)
                                                .description("충전 로봇 배터리"),
                                        fieldWithPath("data[].status").type(JsonFieldType.STRING)
                                                .description("충전 로봇 상태"),
                                        fieldWithPath("data[].reservations[].id").type(JsonFieldType.NUMBER)
                                                .description("예약 아이디"),
                                        fieldWithPath("data[].reservations[].carNumber").type(JsonFieldType.STRING)
                                                .description("예약의 자동차번호"),
                                        fieldWithPath("data[].reservations[].startTime").type(JsonFieldType.STRING)
                                                .description("예약 시작시간"),
                                        fieldWithPath("data[].reservations[].endTime").type(JsonFieldType.STRING)
                                                .description("예약 시작시간"))
                                .build()
                        )
                ));


    }
}
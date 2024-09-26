package com.ssafy.charzzk.api.service.car;

import com.ssafy.charzzk.api.service.car.request.CarServiceRequest;
import com.ssafy.charzzk.api.service.car.response.CarListResponse;
import com.ssafy.charzzk.api.service.car.response.CarResponse;
import com.ssafy.charzzk.api.service.car.response.CarTypeResponse;
import com.ssafy.charzzk.core.exception.BaseException;
import com.ssafy.charzzk.core.exception.ErrorCode;
import com.ssafy.charzzk.domain.car.Car;
import com.ssafy.charzzk.domain.car.CarRepository;
import com.ssafy.charzzk.domain.car.CarType;
import com.ssafy.charzzk.domain.car.CarTypeRepository;
import com.ssafy.charzzk.domain.charginglog.ChargingLog;
import com.ssafy.charzzk.domain.charginglog.ChargingLogRepository;
import com.ssafy.charzzk.domain.user.User;
import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

import java.time.LocalDateTime;
import java.time.temporal.TemporalAdjusters;
import java.util.List;
import java.util.stream.Collectors;

@Transactional(readOnly = true)
@RequiredArgsConstructor
@Service
public class CarService {

    private final CarRepository carRepository;
    private final CarTypeRepository carTypeRepository;
    private final ChargingLogRepository chargingLogRepository;

    @Transactional
    public Long createCar(User user, CarServiceRequest request) {
        CarType carType = carTypeRepository.findById(request.getCarTypeId())
                .orElseThrow(
                        () -> new BaseException(ErrorCode.CAR_TYPE_NOT_FOUND));

        if (carRepository.existsByNumber(request.getNumber())) {
            throw new BaseException(ErrorCode.CAR_NUMBER_ALREADY_EXISTS);
        }

        Car car = Car.create(user, carType, request.getNumber(), request.getNickname());
        carRepository.save(car);
        return car.getId();
    }

    public CarResponse getCar(Long carId) {
        Car findCar = carRepository.findById(carId).orElseThrow(
                () -> new BaseException(ErrorCode.CAR_NOT_FOUND)
        );

        return CarResponse.from(findCar);
    }

    public List<CarTypeResponse> getCarTypes(String name) {
        List<CarType> carTypes = carTypeRepository.findByNameContaining(name);

        return carTypes.stream()
                .map(CarTypeResponse::from)
                .collect(Collectors.toList());
    }

    @Transactional
    public void updateCar(Long carId, User user, CarServiceRequest request) {
        Car car = carRepository.findById(carId).orElseThrow(
                () -> new BaseException(ErrorCode.CAR_NOT_FOUND)
        );

        // 차량의 주인이 맞는지 검증
        if (!car.getUser().getId().equals(user.getId())) {
            throw new BaseException(ErrorCode.CAR_NOT_BELONG_TO_USER);
        }

        // 존재하지 않는 carType 인지 검증
        CarType carType = carTypeRepository.findById(request.getCarTypeId())
                .orElseThrow(
                        () -> new BaseException(ErrorCode.CAR_TYPE_NOT_FOUND));

        // 차량 번호를 수정한다면, 이미 존재하는 차량 번호로 수정하는지 검증
        if (!car.getNumber().equals(request.getNumber())) {
            if (carRepository.existsByNumber(request.getNumber())) {
                throw new BaseException(ErrorCode.CAR_NUMBER_ALREADY_EXISTS);
            }
        }

        car.updateCar(carType, request.getNumber(), request.getNickname());
    }

    @Transactional
    public void deleteCar(Long carId, User user) {

        // 차량 조회
        Car car = carRepository.findById(carId).orElseThrow(
                () -> new BaseException(ErrorCode.CAR_NOT_FOUND)
        );

        // 차량 소유자가 맞는지 검증
        if (!car.getUser().getId().equals(user.getId())) {
            throw new BaseException(ErrorCode.CAR_NOT_BELONG_TO_USER);
        }

        carRepository.delete(car);
    }

    public List<CarListResponse> getCarList(User user) {
        return carRepository.findByUser(user)
                .stream()
                .map(this::convertToCarListResponse)
                .collect(Collectors.toList());
    }

    private CarListResponse convertToCarListResponse(Car car) {
        LocalDateTime startOfMonth = LocalDateTime.now().with(TemporalAdjusters.firstDayOfMonth()).toLocalDate().atStartOfDay();
        LocalDateTime endOfMonth = LocalDateTime.now().with(TemporalAdjusters.lastDayOfMonth()).toLocalDate().atTime(23, 59, 59);

        // TODO: 월말, 월초 경계값 처리 필요 -> 충전 끝나는 시간만 기준으로 하면 될듯함.
        List<ChargingLog> chargingLogs = chargingLogRepository.findByCarAndTimePeriod(car, startOfMonth, endOfMonth);

        long totalChargingTimeInSeconds = chargingLogs.stream()
                .mapToLong(log -> log.getEndTime().getSecond() - log.getStartTime().getSecond())
                .sum();

        // 비용과 충전량 계산 -> 로직 깔끔하게 바꾸긴 해야할 듯함.
        long chargeCost = totalChargingTimeInSeconds * 100;
        long chargeAmount = totalChargingTimeInSeconds * 10;

        return CarListResponse.builder()
                .id(car.getId())
                .carType(CarTypeResponse.from(car.getCarType()))
                .number(car.getNumber())
                .nickname(car.getNickname())
                .isCharging(car.isCharging())
                .chargeCost(chargeCost)
                .chargeAmount(chargeAmount)
                .build();
    }


}

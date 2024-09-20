package com.ssafy.charzzk.api.service.car;

import com.ssafy.charzzk.api.service.car.request.CarServiceRequest;
import com.ssafy.charzzk.api.service.car.response.CarResponse;
import com.ssafy.charzzk.api.service.car.response.CarTypeResponse;
import com.ssafy.charzzk.core.exception.BaseException;
import com.ssafy.charzzk.core.exception.ErrorCode;
import com.ssafy.charzzk.domain.car.Car;
import com.ssafy.charzzk.domain.car.CarRepository;
import com.ssafy.charzzk.domain.car.CarType;
import com.ssafy.charzzk.domain.car.CarTypeRepository;
import com.ssafy.charzzk.domain.user.User;
import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

import java.util.List;
import java.util.stream.Collectors;

@Transactional(readOnly = true)
@RequiredArgsConstructor
@Service
public class CarService {

    private final CarRepository carRepository;
    private final CarTypeRepository carTypeRepository;

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

        CarType carType = carTypeRepository.findById(request.getCarTypeId())
                .orElseThrow(
                        () -> new BaseException(ErrorCode.CAR_TYPE_NOT_FOUND));

        if (carRepository.existsByNumber(request.getNumber())) {
            throw new BaseException(ErrorCode.CAR_NUMBER_ALREADY_EXISTS);
        }

        // TODO: 빈 닉네임이 들어오거나 했을 때, 기존 닉네임을 쓸 수 있도록 해야할듯함.
        car.updateCar(carType, request.getNumber(), request.getNickname());
        carRepository.save(car);
    }
}

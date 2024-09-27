package com.ssafy.charzzk.api.controller.car;

import com.ssafy.charzzk.api.ApiResponse;
import com.ssafy.charzzk.api.controller.car.request.CarRequest;
import com.ssafy.charzzk.api.service.car.CarService;
import com.ssafy.charzzk.api.service.car.response.CarListResponse;
import com.ssafy.charzzk.api.service.car.response.CarResponse;
import com.ssafy.charzzk.api.service.car.response.CarTypeResponse;
import com.ssafy.charzzk.core.annotation.CurrentUser;
import com.ssafy.charzzk.domain.user.User;
import jakarta.validation.Valid;
import lombok.RequiredArgsConstructor;
import org.springframework.web.bind.annotation.*;

import java.time.LocalDateTime;
import java.time.temporal.TemporalAdjusters;
import java.util.List;

@RequiredArgsConstructor
@RestController
public class CarController {

    private final CarService carService;

    @PostMapping("/api/v1/cars")
    public ApiResponse<CarResponse> createCar(
            @CurrentUser User user,
            @Valid @RequestBody CarRequest request
    ) {
        Long carId = carService.createCar(user, request.toServiceRequest());
        return ApiResponse.ok(carService.getCar(carId));
    }

    @GetMapping("/api/v1/car-types")
    public ApiResponse<List<CarTypeResponse>> getCarTypes(
            @RequestParam(value = "q", defaultValue = "", required = false) String keyword
    ) {
        List<CarTypeResponse> carTypes = carService.getCarTypes(keyword);
        return ApiResponse.ok(carTypes);
    }

    @PatchMapping("/api/v1/cars/{carId}")
    public ApiResponse<CarResponse> updateCar(
            @PathVariable Long carId,
            @CurrentUser User user,
            @Valid @RequestBody CarRequest request
    ) {
        carService.updateCar(carId, user, request.toServiceRequest());
        return ApiResponse.ok(carService.getCar(carId));
    }

    @DeleteMapping("/api/v1/cars/{carId}")
    public ApiResponse<CarResponse> deleteCar(
            @PathVariable Long carId,
            @CurrentUser User user
    ) {
        carService.deleteCar(carId, user);
        return ApiResponse.ok(null);
    }

    @GetMapping("/api/v1/cars/me")
    public ApiResponse<List<CarListResponse>> getCarList(
            @CurrentUser User user
    ) {
        LocalDateTime startOfMonth = LocalDateTime.now().with(TemporalAdjusters.firstDayOfMonth()).toLocalDate().atStartOfDay();
        LocalDateTime endOfMonth = LocalDateTime.now().with(TemporalAdjusters.lastDayOfMonth()).toLocalDate().atTime(23, 59, 59);

        List<CarListResponse> carList = carService.getCarList(user, startOfMonth, endOfMonth);
        return ApiResponse.ok(carList);
    }
}

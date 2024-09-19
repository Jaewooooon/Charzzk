package com.ssafy.charzzk.api.controller.car;

import com.ssafy.charzzk.api.ApiResponse;
import com.ssafy.charzzk.api.controller.car.request.CarRequest;
import com.ssafy.charzzk.api.service.car.CarService;
import com.ssafy.charzzk.api.service.car.response.CarResponse;
import com.ssafy.charzzk.core.annotation.CurrentUser;
import com.ssafy.charzzk.domain.user.User;
import jakarta.validation.Valid;
import lombok.RequiredArgsConstructor;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RestController;

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
}

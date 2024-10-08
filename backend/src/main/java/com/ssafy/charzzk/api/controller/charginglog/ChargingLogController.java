package com.ssafy.charzzk.api.controller.charginglog;

import com.ssafy.charzzk.api.ApiResponse;
import com.ssafy.charzzk.api.service.charginglog.ChargingLogService;
import com.ssafy.charzzk.api.service.charginglog.response.ChargingLogResponse;
import com.ssafy.charzzk.core.annotation.CurrentUser;
import com.ssafy.charzzk.domain.user.User;
import lombok.RequiredArgsConstructor;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.RestController;

import java.util.List;

@RequiredArgsConstructor
@RestController
public class ChargingLogController {

    private final ChargingLogService chargingLogService;

    @GetMapping("/api/v1/charging-log")
    public ApiResponse<List<ChargingLogResponse>> getChargingLogList(
            @CurrentUser User user
    ) {
        return ApiResponse.ok(chargingLogService.getChargingLogList(user));
    }

}

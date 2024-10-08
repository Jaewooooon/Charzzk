package com.ssafy.charzzk.api.service.charginglog;

import com.ssafy.charzzk.api.service.charginglog.response.ChargingLogResponse;
import com.ssafy.charzzk.domain.charginglog.ChargingLog;
import com.ssafy.charzzk.domain.charginglog.ChargingLogRepository;
import com.ssafy.charzzk.domain.user.User;
import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

import java.time.Duration;
import java.util.List;
import java.util.stream.Collectors;

import static com.ssafy.charzzk.api.service.car.CarConstant.CHARGE_AMOUNT_PER_HOUR;
import static com.ssafy.charzzk.api.service.car.CarConstant.COST_PER_KWH;

@Transactional(readOnly = true)
@RequiredArgsConstructor
@Service
public class ChargingLogService {

    private final ChargingLogRepository chargingLogRepository;

    public List<ChargingLogResponse> getChargingLogList(User user) {
        List<ChargingLog> chargingLogList = chargingLogRepository.findByUser(user);

        return chargingLogList.stream()
                .map(this::convertToChargingLogResponse)
                .collect(Collectors.toList());
    }

    private ChargingLogResponse convertToChargingLogResponse(ChargingLog chargingLog) {
        long chargingTimeInSeconds = Duration.between(chargingLog.getStartTime(), chargingLog.getEndTime()).getSeconds();
        double chargeAmount = CHARGE_AMOUNT_PER_HOUR * (chargingTimeInSeconds / 3600.0);
        long chargeCost = (long) (chargeAmount * COST_PER_KWH);

        return ChargingLogResponse.of(chargingLog, chargeAmount, chargeCost);
    }

}

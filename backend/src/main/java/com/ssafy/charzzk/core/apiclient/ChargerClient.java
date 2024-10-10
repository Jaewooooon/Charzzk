package com.ssafy.charzzk.core.apiclient;

import com.ssafy.charzzk.core.apiclient.request.ChargerCancelRequest;
import com.ssafy.charzzk.core.apiclient.request.ChargerCommandRequest;
import com.ssafy.charzzk.core.apiclient.response.ChargerCommandResponse;
import org.springframework.beans.factory.annotation.Qualifier;
import org.springframework.cloud.openfeign.FeignClient;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.Mapping;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;

@FeignClient(name = "charger", url = "${embedded.server.url}")
public interface ChargerClient {

    @PostMapping("/charge-command")
    ChargerCommandResponse command(@RequestBody ChargerCommandRequest request);

    @PostMapping("/charge-cancel")
    ChargerCommandResponse cancel(ChargerCancelRequest request);

    @GetMapping("/return-to-start")
    void returnToStart();
}

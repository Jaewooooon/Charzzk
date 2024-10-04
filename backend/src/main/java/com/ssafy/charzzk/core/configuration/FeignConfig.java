package com.ssafy.charzzk.core.configuration;

import org.springframework.cloud.openfeign.EnableFeignClients;
import org.springframework.context.annotation.Configuration;

@Configuration
@EnableFeignClients(basePackages = "com.ssafy.charzzk.core.apiclient")
public class FeignConfig {

}

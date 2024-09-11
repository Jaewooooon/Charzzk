package com.ssafy.charzzk;

import org.springframework.boot.SpringApplication;

public class TestEasyChargeApplication {

	public static void main(String[] args) {
		SpringApplication.from(CharzzkApplication::main).with(TestcontainersConfiguration.class).run(args);
	}

}

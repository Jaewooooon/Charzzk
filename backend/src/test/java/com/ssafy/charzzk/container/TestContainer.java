package com.ssafy.charzzk.container;

import org.springframework.context.annotation.PropertySource;
import org.springframework.test.context.DynamicPropertyRegistry;
import org.springframework.test.context.DynamicPropertySource;
import org.testcontainers.containers.PostgreSQLContainer;
import org.testcontainers.junit.jupiter.Testcontainers;

@Testcontainers
@PropertySource("classpath:application-test.properties")
public class TestContainer {
    private static final String DATABASE_NAME = "charzzk-test-db";
    private static final String USERNAME = "charzzk-test-user";
    private static final String PASSWORD = "charzzk-test-password";
    private static final PostgreSQLContainer<?> postgreSQLContainer;

    static {
        postgreSQLContainer = new PostgreSQLContainer("postgres:16.3-alpine")
                .withDatabaseName(DATABASE_NAME)
                .withUsername(USERNAME)
                .withPassword(PASSWORD);

        postgreSQLContainer.start();

    }

    @DynamicPropertySource
    public static void overrideProps(DynamicPropertyRegistry dynamicPropertyRegistry) {
        dynamicPropertyRegistry.add("spring.datasource.url", postgreSQLContainer::getJdbcUrl);
        dynamicPropertyRegistry.add("spring.datasource.name", () -> DATABASE_NAME);
        dynamicPropertyRegistry.add("spring.datasource.username", () -> USERNAME);
        dynamicPropertyRegistry.add("spring.datasource.password", () -> PASSWORD);
        dynamicPropertyRegistry.add("spring.jpa.hibernate.ddl-auto", () -> "create");
    }
}

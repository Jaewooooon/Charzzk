# Testcontainers

## Testcontainers란 무엇이고, 왜 사용해야 할까?

Testcontainers는 Docker 컨테이너에 래핑된 실제 서비스를 사용하여 통합 테스트를 부트스트래핑하기 위한 쉽고 가벼운 API를 제공하는 테스트 라이브러리입니다.

Testcontainers를 사용하면 모의나 메모리 내 서비스 없이 프로덕션에서 사용하는 것과 동일한 유형의 서비스와 통신하는 테스트를 작성할 수 있습니다.


--- 
기업은 가능한 한 빨리 제품을 시장에 출시하고 피드백을 받고 반복하고 싶어합니다. 

민첩성의 이러한 측면을 달성하려면 견고한 CI/CD(Continuous Integration and Continuous Deployment) 프로세스가 필요합니다. CI/CD 프로세스의 중요한 부분은 애플리케이션 동작의 정확성을 보장하기 위한 자동화된 테스트입니다.


이전까지의 경험을 살펴보면 보통 테스팅을 할 때 로컬데이터베이스에 연결을 하거나
인메모리 데이터베이스를 이용해서 테스트를 진행하는 것이 보편적입니다.

여기에는 몇가지 문제점이 있습니다.

- 메모리 내 서비스는 프로덕션 서비스의 모든 기능을 갖추고 있지 않을 수 있습니다.
  - 예를 들어, 애플리케이션에서 Postgres/Oracle 데이터베이스의 고급 기능을 사용하고 있을 수 있습니다. 하지만 H2는 통합 테스트에 사용하기 위해 모든 기능을 지원하지 않을 수 있습니다
- 메모리 내 서비스는 피드백 주기를 지연시킵니다.
  - 예를 들어, SQL 쿼리를 작성하여 잘 작동하는 H2 메모리 내 데이터베이스로 테스트했을 수 있습니다. 하지만 애플리케이션을 배포한 후 쿼리 구문이 H2에서는 잘 작동하지만 프로덕션 데이터베이스인 Postgres/Oracle에서는 작동하지 않는다는 것을 깨닫게 될 수 있습니다.
---

Spring Boot + Junit5 + Testcontainers를 이용한 통합 테스트 예제를 통한 테스팅 환경 구축



~~~gradle
// build.gradle

testImplementation 'org.springframework.boot:spring-boot-testcontainers'
testImplementation 'org.testcontainers:junit-jupiter'
testImplementation 'org.testcontainers:postgresql'
~~~

~~~java
@Testcontainers
public class TestContainer {
    private static final PostgreSQLContainer<?> postgresContainer = new PostgreSQLContainer<>("postgres:16.3-alpine")
            .withDatabaseName("testdb")
            .withUsername("test")
            .withPassword("test");

    static {
        postgresContainer.start();
    }

    @DynamicPropertySource
    static void postgresProperties(DynamicPropertyRegistry registry) {
        registry.add("spring.datasource.url", postgresContainer::getJdbcUrl);
        registry.add("spring.datasource.username", postgresContainer::getUsername);
        registry.add("spring.datasource.password", postgresContainer::getPassword);
    }
}
~~~

~~~java
@DataJpaTest
class UserRepositoryTest extends TestContainer {

    @Autowired
    private UserRepository userRepository;

    @DisplayName("유저 목록 조회")
    @Test
    void getUserList() {
        // given
        List<User> users = List.of(
                User.create("test_user1"),
                User.create("test_user2"),
                User.create("test_user3")
        );
        userRepository.saveAll(users);

        // when
        List<User> findUsers = userRepository.findAll();

        // then
        assertThat(findUsers.size()).isEqualTo(3);
    }
}
~~~
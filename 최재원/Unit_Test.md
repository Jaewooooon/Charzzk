### 테스트코드를 왜 작성해야 할까?

단위 테스트를 작성해야 하는 이유는 정말 너무 많다. 그 중에서 몇 가지 핵심적인 이유들을 작성하면 다음과 같다.

1. **코드를 수정하거나 기능을 추가할 때 수시로 빠르게 검증 할 수 있다.**
2. **리팩토링 시에 안정성을 확보할 수 있다.**
3. **개발 및 테스팅에 대한 시간과 비용을 절감할 수 있다.**

처음에는 시간도 오래걸리고 굳이 작성해야 하나 생각이 들 수 있지만 습관을 들이다 보면 분명히 장점들에 대해 느낄 수 있을 것!

## 더 나은 테스트를 작성하려면?

### 한 문단에 한 주제만 작성

테스트코드는 그 자체로 문서로서의 기능을 한다. 

테스트 시나리오를 보고 코드가 어떻게 동작하는지 알 수 있어야 함!

예를 들어 if문으로 분기를 나눴으면 하나의 테스트에서 전부 검증하지 말고 경우에 따라 나눠서 쓰도록 하자.

—> @DIsplayName을 한 문장으로 만들 수 있도록 테스트를 작성하자

### 완벽하게 제어하기

현재 시간, 랜덤 값 등 제어할 수 없는 값들은 상위 계층으로 분리해서 테스트 가능한 구조로 만들 수 있다.

테스트를 할 땐 원하는 시간을 인자로 받아서 테스트 할 수 있도록!

### 테스트의 독립성 보장하기

두 가지 이상의 테스트가 하나의 자원을 공유하지 말자

테스트 사이의 순서는 결과와 무관해야 한다. 각각 독립적으로 실행되어 언제든지 올바른 결과를 내야 함. 

### 한 눈에 들어오는 TestFixture 구성하기

TestFixture : 테스트를 위해 원하는 상태로 고정시킨 (given에서 생성한)객체

@BeforeAll: 테스트 클래스 전체 실행 전에 한 번 실행

@BeforeEach: 매 테스트 메서드 실행 전에 실행

같은 fixture를 공유하는 테스트의 코드들을 중복제거하기 위해 @BeforeEach에 한번에 몰아넣고 싶을 수 있지만, 테스트간 결합도를 높이기 때문에 사용하지 않는 것이 좋다.

그럼 @BeforeEach는 언제 쓸까?

- 각 테스트 입장에서 몰라도 테스트 내용을 이해하는데 문제가 없을 때
- 수정해도 모든 테스트에 영향을 주지 않을 때

ex) 어떤 엔티티를 생성할 때 필요한 엔티티가 있는데 테스트할 땐 필요 없는 경우

given절이 길어지더라도 문서로서의 테스트를 생각하면서 작성하도록 하자

테스트에 딱 필요한 필드만 명시적으로 생성할 수 있는 함수를 테스트 클래스에 만들어서 사용하자

### TestFixture 클렌징

@DeleteAll과 @DeleteAllInBatch의 차이점

**@DeleteAllInBatch**

데이터를 한번에 벌크성으로 지워줌

다대다관계에서 외래키 예외 때문에 매핑 테이블을 먼저 지워야만 함

순서를 잘 고려해야 하는 불편함

@DeleteAll

외래키를 맺고 있는 애들을 하나씩 select해와서 지워줌. 매핑테이블을 같이 지워줌

그러나 쿼리의 수가 너무 많아서 

그럼에도 **@DeleteAllInBatch**를 쓰는 이유?

순서를 고려하는 불편함만 신경쓰면 성능상의 이점이 있다

### @ParameterizedTest

하나의 테스트케이스에서 에서 값을 여러개로 바꿔가며 테스트하고 싶을 때

그래서 값이나 어떤 환경에 대한 그런 데이터들을 바꿔가면서 테스트를 여러 번 반복을 하고 싶을 때 케이스를 확장

```java
    @DisplayName("상품 타입이 재고 관련 타입인지를 체크한다.")
    @CsvSource({"HANDMADE,false","BOTTLE,true","BAKERY,true"})
    @ParameterizedTest
    void containsStockType4(ProductType productType, boolean expected) {
        // when
        boolean result = ProductType.containsStockType(productType);

        // then
        assertThat(result).isEqualTo(expected);
    }
```

```java
    
    
    @DisplayName("상품 타입이 재고 관련 타입인지를 체크한다.")
    @MethodSource("provideProductTypesForCheckingStockType")
    @ParameterizedTest
    void containsStockType5(ProductType productType, boolean expected) {
        // when
        boolean result = ProductType.containsStockType(productType);

        // then
        assertThat(result).isEqualTo(expected);
    }
    
   private static Stream<Arguments> provideProductTypesForCheckingStockType() {
        return Stream.of(
            Arguments.of(ProductType.HANDMADE, false),
            Arguments.of(ProductType.BOTTLE, true),
            Arguments.of(ProductType.BAKERY, true)
        );
    }
```

여러지 소스들을 공식문서에서 확인 가능

### 테스트 수행도 비용! 테스트 환경 통합하기

테스트가 많을 수록 서버가 많이 뜨게 됨 → 오랜 시간 소요

어떻게 하면 더 빠른 시간 안에 더 효율적으로 테스트를 할 수 있을까?

공통적인 환경을 모아서 서버가 뜨는 횟수를 줄여보자

테스트서포트를 상속받아 동일한 환경 만들어주기

### private 메서드의 테스트는 어떻게 하나요?

할 필요가 없고 하려고 해서도 안된다.

왜냐하면 공개된 api들을 테스트하다보면 자연스럽게 그 안에 있는 private 메서드들이 검증이 된다.

그럼에도 그런 생각이 강하게 든다면?

—> 하나의 메서드에서 하고 있는 일이 너무 많나? 객체를 분리할 시점이 아닌가 생각해보자.

### 테스트에서만 필요한 메서드가 생겼는데 프로덕션 코드에서 필요없다면?

만들어도 된다! 컨트롤러 테스트를 위한 DTO의 생성자 등..

다만 보수적으로 접근하는 것이 좋고 막 만들어내는 것은 지양해야 함!
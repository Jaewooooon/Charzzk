package com.ssafy.charzzk.core.util;

import jakarta.servlet.http.Cookie;
import jakarta.servlet.http.HttpServletRequest;
import jakarta.servlet.http.HttpServletResponse;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.mockito.Mockito;

import java.util.Optional;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.api.Assertions.*;

class CookieUtilsTest {
    private HttpServletRequest request;
    private HttpServletResponse response;

    @BeforeEach
    void setUp() {
        request = Mockito.mock(HttpServletRequest.class);
        response = Mockito.mock(HttpServletResponse.class);
    }

    @DisplayName("쿠키가 없을 경우 Optional.empty()를 반환한다.")
    @Test
    void getCookieShouldReturnEmptyOptionalWhenNoCookies() {
        // given
        Mockito.when(request.getCookies()).thenReturn(null);

        // when
        Optional<Cookie> cookie = CookieUtils.getCookie(request, "testCookie");

        // then
        assertThat(cookie).isEmpty();
    }

    @DisplayName("쿠키 배열이 비어 있을 경우 Optional.empty()를 반환한다.")
    @Test
    void getCookieShouldReturnEmptyOptionalWhenCookiesArrayIsEmpty() {
        // given
        Cookie[] cookies = new Cookie[0]; // 빈 쿠키 배열
        Mockito.when(request.getCookies()).thenReturn(cookies);

        // when
        Optional<Cookie> cookie = CookieUtils.getCookie(request, "testCookie");

        // then
        assertThat(cookie).isEmpty();
    }

    @DisplayName("쿠키가 존재할 경우 해당 쿠키를 반환한다.")
    @Test
    void getCookieShouldReturnCookieWhenPresent() {
        // given
        Cookie cookie = new Cookie("testCookie", "testValue");
        Cookie[] cookies = new Cookie[]{cookie};
        Mockito.when(request.getCookies()).thenReturn(cookies);

        // when
        Optional<Cookie> result = CookieUtils.getCookie(request, "testCookie");

        // then
        assertThat(result).isPresent();
        assertThat(result.get().getName()).isEqualTo("testCookie");
        assertThat(result.get().getValue()).isEqualTo("testValue");
    }

    @DisplayName("쿠키 배열에 여러 개의 쿠키가 있을 경우 특정 쿠키를 반환한다.")
    @Test
    void getCookieShouldReturnCookieWhenMultipleCookiesArePresent() {
        // given
        Cookie cookie1 = new Cookie("cookie1", "value1");
        Cookie cookie2 = new Cookie("testCookie", "testValue");
        Cookie cookie3 = new Cookie("cookie3", "value3");
        Cookie[] cookies = new Cookie[]{cookie1, cookie2, cookie3};
        Mockito.when(request.getCookies()).thenReturn(cookies);

        // when
        Optional<Cookie> result = CookieUtils.getCookie(request, "testCookie");

        // then
        assertThat(result).isPresent();
        assertThat(result.get().getName()).isEqualTo("testCookie");
        assertThat(result.get().getValue()).isEqualTo("testValue");
    }

    @DisplayName("쿠키를 추가할 때 response에 쿠키가 추가된다.")
    @Test
    void addCookieShouldAddCookieToResponse() {
        // given
        String name = "testCookie";
        String value = "testValue";
        int maxAge = 3600;
        boolean httpOnly = true;

        // when
        CookieUtils.addCookie(response, name, value, maxAge, httpOnly);

        // then
        Mockito.verify(response).addCookie(Mockito.argThat(cookie ->
                cookie.getName().equals(name) &&
                        cookie.getValue().equals(value) &&
                        cookie.getMaxAge() == maxAge &&
                        cookie.isHttpOnly() == httpOnly
        ));
    }

    @DisplayName("쿠키를 제거할 때 response에 삭제된 쿠키가 추가된다.")
    @Test
    void removeCookieShouldAddExpiredCookieToResponse() {
        // given
        String name = "testCookie";

        // when
        CookieUtils.removeCookie(response, name);

        // then
        Mockito.verify(response).addCookie(Mockito.argThat(cookie ->
                cookie.getName().equals(name) &&
                        cookie.getMaxAge() == 0
        ));
    }

    @DisplayName("객체를 직렬화하여 문자열로 변환한다.")
    @Test
    void serializeShouldReturnSerializedString() {
        // given
        String originalValue = "testValue";

        // when
        String serializedValue = CookieUtils.serialize(originalValue);

        // then
        assertThat(serializedValue).isNotNull();
        assertThat(serializedValue).isNotEmpty();
    }

    @DisplayName("직렬화된 문자열로부터 객체를 역직렬화한다.")
    @Test
    void deserializeShouldReturnDeserializedObject() {
        // given
        String originalValue = "testValue";
        String serializedValue = CookieUtils.serialize(originalValue);
        Cookie cookie = new Cookie("testCookie", serializedValue);

        // when
        String deserializedValue = CookieUtils.deserialize(cookie, String.class);

        // then
        assertThat(deserializedValue).isEqualTo(originalValue);
    }

    @DisplayName("기본생성자 테스트")
    @Test
    public void noArgsConstructor() {
        new CookieUtils();
    }
}

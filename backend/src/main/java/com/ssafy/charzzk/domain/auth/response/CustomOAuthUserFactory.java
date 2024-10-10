package com.ssafy.charzzk.domain.auth.response;

import com.ssafy.charzzk.domain.auth.response.impl.GoogleResponse;
import com.ssafy.charzzk.domain.auth.response.impl.KakaoResponse;
import com.ssafy.charzzk.domain.auth.response.impl.NaverResponse;

import java.util.Map;

public class CustomOAuthUserFactory {
    public static OAuth2Response parseOAuth2Response(ProviderType providerType, Map<String, Object> attributes) {
        return switch (providerType) {
            case NAVER -> new NaverResponse(attributes);
            case GOOGLE -> new GoogleResponse(attributes);
            case KAKAO -> new KakaoResponse(attributes);
        };
    }
}

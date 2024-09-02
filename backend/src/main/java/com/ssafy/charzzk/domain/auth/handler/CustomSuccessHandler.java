package com.ssafy.charzzk.domain.auth.handler;


import com.ssafy.charzzk.domain.auth.AuthConst;
import com.ssafy.charzzk.domain.auth.CustomAuthorizationRequestRepository;
import com.ssafy.charzzk.domain.auth.response.CustomOAuth2User;
import com.ssafy.charzzk.api.service.auth.JWTService;
import com.ssafy.charzzk.domain.auth.jwt.JwtProperties;
import com.ssafy.charzzk.domain.auth.jwt.JwtToken;
import com.ssafy.charzzk.core.util.CookieUtils;
import jakarta.servlet.http.Cookie;
import jakarta.servlet.http.HttpServletRequest;
import jakarta.servlet.http.HttpServletResponse;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.security.core.Authentication;
import org.springframework.security.web.authentication.SimpleUrlAuthenticationSuccessHandler;
import org.springframework.stereotype.Component;

import java.io.IOException;
import java.util.Optional;

import static com.ssafy.charzzk.domain.auth.AuthConst.REDIRECT_URI_PARAM_COOKIE_NAME;


@Slf4j
@Component
@RequiredArgsConstructor
public class CustomSuccessHandler extends SimpleUrlAuthenticationSuccessHandler {
    private final JWTService jwtService;
    private final JwtProperties properties;
    private final CustomAuthorizationRequestRepository customAuthorizationRequestRepository;

    @Override
    protected String determineTargetUrl(HttpServletRequest request, HttpServletResponse response, Authentication authentication) {
        Optional<String> redirectUri = CookieUtils.getCookie(request, REDIRECT_URI_PARAM_COOKIE_NAME).map(Cookie::getValue);
        clearAuthenticationAttributes(request, response);
        return redirectUri.orElse(getDefaultTargetUrl());
    }

    @Override
    public void onAuthenticationSuccess(HttpServletRequest request, HttpServletResponse response, Authentication authentication)
            throws IOException {
        CustomOAuth2User customUserDetails = (CustomOAuth2User) authentication.getPrincipal();

        JwtToken jwtToken = jwtService.generateToken(customUserDetails.getUsername(), customUserDetails.getAuthorities());
        CookieUtils.addCookie(response, AuthConst.REFRESH_TOKEN, jwtToken.getRefreshToken(), properties.getRefreshExpire(), true);

        response.setHeader("Authorization", "Bearer " + jwtToken.getAccessToken());

        String redirectURI = determineTargetUrl(request, response, authentication);
        getRedirectStrategy().sendRedirect(request, response, redirectURI);
    }

    protected void clearAuthenticationAttributes(HttpServletRequest request, HttpServletResponse response) {
        customAuthorizationRequestRepository.removeAuthorizationRequestCookies(request, response);
    }
}

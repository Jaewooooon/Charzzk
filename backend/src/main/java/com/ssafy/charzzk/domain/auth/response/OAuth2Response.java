package com.ssafy.charzzk.domain.auth.response;


public interface OAuth2Response {
	ProviderType getProvider();

	String getProviderId();

	String getEmail();

	String getName();

	String getGender();
}

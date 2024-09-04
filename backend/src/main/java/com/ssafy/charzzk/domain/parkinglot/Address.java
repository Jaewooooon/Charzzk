package com.ssafy.charzzk.domain.parkinglot;

import jakarta.persistence.Column;
import jakarta.persistence.Embeddable;
import lombok.AccessLevel;
import lombok.NoArgsConstructor;

@Embeddable
@NoArgsConstructor(access = AccessLevel.PROTECTED)
public class Address {

    @Column(nullable = false)
    private String city;

    @Column(nullable = false)
    private String district;

    @Column(nullable = false)
    private String detail;

    public Address(String city, String district, String detail) {
        this.city = city;
        this.district = district;
        this.detail = detail;
    }
}

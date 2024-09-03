package com.ssafy.charzzk.domain.parkinglot;

import jakarta.persistence.Embeddable;

@Embeddable
public class Address {

    private String city;

    private String district;

    private String detail;

    public Address() {}

    public Address(String city, String district, String detail) {
        this.city = city;
        this.district = district;
        this.detail = detail;
    }
}

package com.ssafy.charzzk.domain.parkinglot;

import jakarta.persistence.Embeddable;

@Embeddable
public class Location {
    private Double latitude;

    private Double longitude;

    public Location() {
    }

    public Location(Double latitude, Double longitude) {
        this.latitude = latitude;
        this.longitude = longitude;
    }
}

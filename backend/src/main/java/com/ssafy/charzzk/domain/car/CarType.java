package com.ssafy.charzzk.domain.car;

import jakarta.persistence.*;
import lombok.AccessLevel;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Entity
@Getter
@NoArgsConstructor(access = AccessLevel.PROTECTED)
public class CarType {

    @Id
    @Column(name = "car_type_id")
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    private Long id;

    @Column(nullable = false)
    private String name;

    // 이미지 필수일까?
    private String image;

    @Builder
    private CarType(String name, String image) {
        this.name = name;
        this.image = image;
    }

    public static CarType create(String name, String image) {
        return CarType.builder()
                .name(name)
                .image(image)
                .build();
    }
}

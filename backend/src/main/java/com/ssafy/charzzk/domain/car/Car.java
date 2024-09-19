package com.ssafy.charzzk.domain.car;

import com.ssafy.charzzk.domain.BaseEntity;
import com.ssafy.charzzk.domain.user.User;
import jakarta.persistence.*;
import lombok.AccessLevel;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Entity
@Getter
@NoArgsConstructor(access = AccessLevel.PROTECTED)
public class Car extends BaseEntity {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    @Column(name = "car_id")
    private Long id;

    @ManyToOne(fetch = FetchType.LAZY)
    @JoinColumn(name = "user_id", nullable = false)
    private User user;

    @ManyToOne(fetch = FetchType.LAZY)
    @JoinColumn(name = "car_type_id", nullable = false)
    private CarType carType;

    @Column(nullable = false)
    private String number;

    private String nickname;

    @Builder
    public Car(User user, CarType carType, String number, String nickname) {
        this.user = user;
        this.carType = carType;
        this.number = number;
        this.nickname = nickname;
    }

    public static Car create(User user, CarType carType, String number, String nickname) {
        return Car.builder()
                .user(user)
                .carType(carType)
                .number(number)
                .nickname(nickname)
                .build();

    }
}

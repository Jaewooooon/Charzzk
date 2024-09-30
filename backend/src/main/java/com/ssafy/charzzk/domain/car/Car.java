package com.ssafy.charzzk.domain.car;

import com.ssafy.charzzk.domain.BaseEntity;
import com.ssafy.charzzk.domain.user.User;
import jakarta.persistence.*;
import lombok.AccessLevel;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Entity
@Table(
        uniqueConstraints = {@UniqueConstraint(columnNames = "number")}
)
@Getter
@NoArgsConstructor(access = AccessLevel.PROTECTED)
public class Car extends BaseEntity {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    @Column(name = "car_id")
    private Long id;

    @ManyToOne(fetch = FetchType.LAZY, cascade = CascadeType.ALL)
    @JoinColumn(name = "user_id", nullable = false)
    private User user;

    @ManyToOne(fetch = FetchType.LAZY, cascade = CascadeType.ALL)
    @JoinColumn(name = "car_type_id", nullable = false)
    private CarType carType;

    @Column(nullable = false)
    private String number;

    private String nickname;

    private int battery = 30;

    @Builder
    private Car(Long id, User user, CarType carType, String number, String nickname, int battery) {
        this.id = id;
        this.user = user;
        this.carType = carType;
        this.number = number;
        this.nickname = nickname;
        this.battery = battery;
    }

    public static Car create(User user, CarType carType, String number, String nickname) {
        return Car.builder()
                .user(user)
                .carType(carType)
                .number(number)
                .nickname(nickname)
                .build();
    }

    public void updateCar(CarType carType, String number, String nickname) {
        this.carType = carType;
        this.number = number;
        this.nickname = nickname;
    }

}

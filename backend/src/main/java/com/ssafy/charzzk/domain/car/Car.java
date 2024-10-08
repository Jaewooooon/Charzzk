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

    @ManyToOne(fetch = FetchType.LAZY, cascade = CascadeType.PERSIST)
    @JoinColumn(name = "user_id", nullable = false)
    private User user;

    @ManyToOne(fetch = FetchType.LAZY, cascade = CascadeType.PERSIST)
    @JoinColumn(name = "car_type_id", nullable = false)
    private CarType carType;

    @Column(nullable = false)
    private String number;

    @Column(nullable = true)
    private String nickname;

    @Column(nullable = false)
    private int battery = 30;

    @Column(nullable = false)
    private boolean isCharging;

    @Builder
    private Car(Long id, User user, CarType carType, String number, String nickname, int battery, boolean isCharging) {
        this.id = id;
        this.user = user;
        this.carType = carType;
        this.number = number;
        this.nickname = nickname;
        this.battery = battery;
        this.isCharging = isCharging;
    }

    public static Car create(User user, CarType carType, String number, String nickname) {
        return Car.builder()
                .user(user)
                .carType(carType)
                .number(number)
                .nickname(nickname)
                .battery(30)
                .isCharging(false)
                .build();
    }

    public void updateCar(CarType carType, String number, String nickname) {
        this.carType = carType;
        this.number = number;
        this.nickname = nickname;
    }

    public void chargeBattery(Integer batteryLevel) {
        this.battery = batteryLevel;
    }

    public void chargeComplete() {
        this.isCharging = false;
    }

    public void startCharge() {
        this.isCharging = true;
    }
}

package com.ssafy.charzzk.domain.order;

import com.ssafy.charzzk.domain.charginglog.ChargingLog;
import jakarta.persistence.*;
import lombok.AccessLevel;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Entity
@Getter
@Table(name = "orders")
@NoArgsConstructor(access = AccessLevel.PROTECTED)
public class Order {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    @Column(name = "order_id")
    private Long id;

    @OneToOne(fetch = FetchType.LAZY)
    @JoinColumn(name = "charging_log_id", nullable = false)
    private ChargingLog chargingLog;

    private Long cost;

    @Builder
    private Order(ChargingLog chargingLog, Long cost) {
        this.chargingLog = chargingLog;
        this.cost = cost;
    }
}

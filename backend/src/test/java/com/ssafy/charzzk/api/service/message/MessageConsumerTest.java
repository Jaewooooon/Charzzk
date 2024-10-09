package com.ssafy.charzzk.api.service.message;

import com.ssafy.charzzk.api.service.message.request.MessageRequest;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import static org.mockito.ArgumentMatchers.anyInt;
import static org.mockito.ArgumentMatchers.anyLong;
import static org.mockito.Mockito.doThrow;
import static org.mockito.Mockito.verify;


@ExtendWith(MockitoExtension.class)
class MessageConsumerTest {

    @Mock
    private MessageService messageService;

    @InjectMocks
    private MessageConsumer messageConsumer;

    @DisplayName("인식 상태 메시지를 처리한다.")
    @Test
    void shouldHandleRecognitionStatusMessage() {
        // given
        MessageRequest request = MessageRequest.builder()
                .type("recognition_status")
                .build();

        // when
        messageConsumer.receiveMessage(request);

        // then

        // 인식 상태 처리 로직을 추가해야 하므로 별도의 검증 로직 필요
        // 현재는 처리 로직이 없으므로 그냥 호출 확인
    }

    @DisplayName("배터리 상태 메시지를 처리한다.")
    @Test
    void shouldHandleBatteryStatusMessage() {
        // given
        MessageRequest request = MessageRequest.builder()
                .type("battery_status")
                .reservationId(1L)
                .batteryLevel(75)
                .build();

        // when
        messageConsumer.receiveMessage(request);

        // then
        verify(messageService).updateBatteryStatus(1L, 75);
    }

    @DisplayName("충전 완료 메시지를 처리한다.")
    @Test
    void shouldHandleChargeCompleteMessage() {
        // given
        MessageRequest request = MessageRequest.builder()
                .type("charge_complete")
                .reservationId(1L)
                .build();

        // when
        messageConsumer.receiveMessage(request);

        // then
        verify(messageService).chargeComplete(1L);
    }

    @DisplayName("위치 업데이트 메시지를 처리한다.")
    @Test
    void shouldHandleLocationUpdateMessage() {
        // given
        MessageRequest request = MessageRequest.builder()
                .type("location_update")
                .build();

        // when
        messageConsumer.receiveMessage(request);

    }

    @DisplayName("알 수 없는 메시지 유형을 처리한다.")
    @Test
    void shouldHandleUnknownMessageType() {
        // given
        MessageRequest request = MessageRequest.builder()
                .type("unknown_type")
                .build();

        // when
        messageConsumer.receiveMessage(request);

    }

    @DisplayName("예외 발생 시 메시지 처리에 실패한다.")
    @Test
    void shouldHandleExceptionWhenProcessingMessage() {
        // given
        MessageRequest request = MessageRequest.builder()
                .type("battery_status")
                .reservationId(1L)
                .batteryLevel(75)
                .build();

        // when
        doThrow(new RuntimeException("Test Exception")).when(messageService).updateBatteryStatus(anyLong(), anyInt());
        messageConsumer.receiveMessage(request);

        // then
        verify(messageService).updateBatteryStatus(1L, 75);
    }
}

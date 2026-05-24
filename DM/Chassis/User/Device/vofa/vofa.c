#include "vofa.h"

void USART_Vofa_Justfloat_Transmit(float SendValue1, float SendValue2) {

    __attribute__((section (".AXI_SRAM")))  static uint8_t Rx_Buf[12];

    uint8_t *SendValue1_Pointer, *SendValue2_Pointer;

    SendValue1_Pointer = (uint8_t *) &SendValue1;
    SendValue2_Pointer = (uint8_t *) &SendValue2;


    Rx_Buf[0] = *SendValue1_Pointer;
    Rx_Buf[1] = *(SendValue1_Pointer + 1);
    Rx_Buf[2] = *(SendValue1_Pointer + 2);
    Rx_Buf[3] = *(SendValue1_Pointer + 3);
    Rx_Buf[4] = *SendValue2_Pointer;
    Rx_Buf[5] = *(SendValue2_Pointer + 1);
    Rx_Buf[6] = *(SendValue2_Pointer + 2);
    Rx_Buf[7] = *(SendValue2_Pointer + 3);
    Rx_Buf[8] = 0x00;
    Rx_Buf[9] = 0x00;
    Rx_Buf[10] = 0x80;
    Rx_Buf[11] = 0x7F;

    HAL_UART_Transmit_DMA(&huart7, Rx_Buf, sizeof(Rx_Buf));
}

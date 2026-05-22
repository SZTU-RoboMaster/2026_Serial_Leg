#include "vofa.h"

void USART_Vofa_Justfloat_Transmit(
    float SendValue1,  float SendValue2,  float SendValue3,
    float SendValue4,  float SendValue5,  float SendValue6,
    float SendValue7,  float SendValue8,  float SendValue9,
    float SendValue10, float SendValue11, float SendValue12,
    float SendValue13, float SendValue14, float SendValue15,
    float SendValue16, float SendValue17, float SendValue18,
    float SendValue19, float SendValue20, float SendValue21,
    float SendValue22, float SendValue23, float SendValue24,
    float SendValue25, float SendValue26, float SendValue27,
    float SendValue28, float SendValue29, float SendValue30,
    float SendValue31, float SendValue32, float SendValue33,
    float SendValue34, float SendValue35, float SendValue36,
    float SendValue37, float SendValue38, float SendValue39,
    float SendValue40, float SendValue41, float SendValue42,
    float SendValue43, float SendValue44, float SendValue45,
    float SendValue46, float SendValue47, float SendValue48,
    float SendValue49, float SendValue50, float SendValue51,
    float SendValue52, float SendValue53, float SendValue54,
    float SendValue55, float SendValue56, float SendValue57,
    float SendValue58, float SendValue59, float SendValue60)
{
    __attribute__((section (".AXI_SRAM")))
    static uint8_t Tx_Buf[244];  // 60*4 + 4 = 244字节

    float *value_array[] = {
        &SendValue1,  &SendValue2,  &SendValue3,
        &SendValue4,  &SendValue5,  &SendValue6,
        &SendValue7,  &SendValue8,  &SendValue9,
        &SendValue10, &SendValue11, &SendValue12,
        &SendValue13, &SendValue14, &SendValue15,
        &SendValue16, &SendValue17, &SendValue18,
        &SendValue19, &SendValue20, &SendValue21,
        &SendValue22, &SendValue23, &SendValue24,
        &SendValue25, &SendValue26, &SendValue27,
        &SendValue28, &SendValue29, &SendValue30,
        &SendValue31, &SendValue32, &SendValue33,
        &SendValue34, &SendValue35, &SendValue36,
        &SendValue37, &SendValue38, &SendValue39,
        &SendValue40, &SendValue41, &SendValue42,
        &SendValue43, &SendValue44, &SendValue45,
        &SendValue46, &SendValue47, &SendValue48,
        &SendValue49, &SendValue50, &SendValue51,
        &SendValue52, &SendValue53, &SendValue54,
        &SendValue55, &SendValue56, &SendValue57,
        &SendValue58, &SendValue59, &SendValue60
    };

    // 批量拷贝数据
    for(int i = 0; i < 60; i++) {
        uint8_t *ptr = (uint8_t*)value_array[i];
        Tx_Buf[i*4 + 0] = ptr[0];
        Tx_Buf[i*4 + 1] = ptr[1];
        Tx_Buf[i*4 + 2] = ptr[2];
        Tx_Buf[i*4 + 3] = ptr[3];
    }

    // VOFA+ JustFloat协议结束标志
    Tx_Buf[240] = 0x00;
    Tx_Buf[241] = 0x00;
    Tx_Buf[242] = 0x80;
    Tx_Buf[243] = 0x7F;

    HAL_UART_Transmit_DMA(&huart7, Tx_Buf, sizeof(Tx_Buf));
}
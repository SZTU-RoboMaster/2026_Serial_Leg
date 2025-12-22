#ifndef _BSP_USART_H
#define _BSP_USART_H

#include "usart.h"
#include "remote.h"

void USART_RxDMA_MultiBuffer_Init(UART_HandleTypeDef *huart, uint32_t *DstAddress, uint32_t *SecondMemAddress,
                                  uint32_t DataLength);

extern uint8_t SBUS_MultiRx_Buf[2][SBUS_RX_BUF_NUM];

#endif

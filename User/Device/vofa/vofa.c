#include "vofa.h"

void USART_Vofa_Justfloat_Transmit(float SendValue1,float SendValue2,float SendValue3,float SendValue4,float SendValue5,float SendValue6,float SendValue7,float SendValue8,float SendValue9){

	__attribute__((section (".AXI_SRAM")))

	static uint8_t Rx_Buf[40];

	uint8_t *SendValue1_Pointer, *SendValue2_Pointer, *SendValue3_Pointer;
	uint8_t *SendValue4_Pointer, *SendValue5_Pointer, *SendValue6_Pointer;
	uint8_t *SendValue7_Pointer, *SendValue8_Pointer, *SendValue9_Pointer;

	SendValue1_Pointer = (uint8_t *)&SendValue1;
	SendValue2_Pointer = (uint8_t *)&SendValue2;
	SendValue3_Pointer = (uint8_t *)&SendValue3;
	SendValue4_Pointer = (uint8_t *)&SendValue4;
	SendValue5_Pointer = (uint8_t *)&SendValue5;
	SendValue6_Pointer = (uint8_t *)&SendValue6;
	SendValue7_Pointer = (uint8_t *)&SendValue7;
	SendValue8_Pointer = (uint8_t *)&SendValue8;
	SendValue9_Pointer = (uint8_t *)&SendValue9;

	Rx_Buf[0] =  *SendValue1_Pointer;
	Rx_Buf[1] =  *(SendValue1_Pointer + 1);
	Rx_Buf[2] =  *(SendValue1_Pointer + 2);
	Rx_Buf[3] =  *(SendValue1_Pointer + 3);
	Rx_Buf[4] =  *SendValue2_Pointer;
	Rx_Buf[5] =  *(SendValue2_Pointer + 1);
	Rx_Buf[6] =  *(SendValue2_Pointer + 2);
	Rx_Buf[7] =  *(SendValue2_Pointer + 3);
	Rx_Buf[8] =  *SendValue3_Pointer;
	Rx_Buf[9] =  *(SendValue3_Pointer + 1);
	Rx_Buf[10] = *(SendValue3_Pointer + 2);
	Rx_Buf[11] = *(SendValue3_Pointer + 3);
	Rx_Buf[12] = *SendValue4_Pointer;
	Rx_Buf[13] = *(SendValue4_Pointer + 1);
	Rx_Buf[14] = *(SendValue4_Pointer + 2);
	Rx_Buf[15] = *(SendValue4_Pointer + 3);
	Rx_Buf[16] = *SendValue5_Pointer;
	Rx_Buf[17] = *(SendValue5_Pointer + 1);
	Rx_Buf[18] = *(SendValue5_Pointer + 2);
	Rx_Buf[19] = *(SendValue5_Pointer + 3);
	Rx_Buf[20] = *SendValue6_Pointer;
	Rx_Buf[21] = *(SendValue6_Pointer + 1);
	Rx_Buf[22] = *(SendValue6_Pointer + 2);
	Rx_Buf[23] = *(SendValue6_Pointer + 3);
	Rx_Buf[24] = *SendValue7_Pointer;
	Rx_Buf[25] = *(SendValue7_Pointer + 1);
	Rx_Buf[26] = *(SendValue7_Pointer + 2);
	Rx_Buf[27] = *(SendValue7_Pointer + 3);
	Rx_Buf[28] = *SendValue8_Pointer;
	Rx_Buf[29] = *(SendValue8_Pointer + 1);
	Rx_Buf[30] = *(SendValue8_Pointer + 2);
	Rx_Buf[31] = *(SendValue8_Pointer + 3);
	Rx_Buf[32] = *SendValue9_Pointer;
	Rx_Buf[33] = *(SendValue9_Pointer + 1);
	Rx_Buf[34] = *(SendValue9_Pointer + 2);
	Rx_Buf[35] = *(SendValue9_Pointer + 3);

	Rx_Buf[36] =  0x00;
	Rx_Buf[37] =  0x00;
	Rx_Buf[38] =  0x80;
	Rx_Buf[39] =  0x7F;

	HAL_UART_Transmit_DMA(&huart7,Rx_Buf,sizeof(Rx_Buf));

}
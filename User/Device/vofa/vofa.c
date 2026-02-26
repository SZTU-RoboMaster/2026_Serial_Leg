#include "vofa.h"
//
// void USART_Vofa_Justfloat_Transmit(float SendValue1, float SendValue2, float SendValue3) {
//
//     __attribute__((section (".AXI_SRAM")))  static uint8_t Rx_Buf[16];
//
//     uint8_t *SendValue1_Pointer, *SendValue2_Pointer, *SendValue3_Pointer;
//
//     SendValue1_Pointer = (uint8_t *) &SendValue1;
//     SendValue2_Pointer = (uint8_t *) &SendValue2;
//     SendValue3_Pointer = (uint8_t *) &SendValue3;
//
//
//     Rx_Buf[0] = *SendValue1_Pointer;
//     Rx_Buf[1] = *(SendValue1_Pointer + 1);
//     Rx_Buf[2] = *(SendValue1_Pointer + 2);
//     Rx_Buf[3] = *(SendValue1_Pointer + 3);
//     Rx_Buf[4] = *SendValue2_Pointer;
//     Rx_Buf[5] = *(SendValue2_Pointer + 1);
//     Rx_Buf[6] = *(SendValue2_Pointer + 2);
//     Rx_Buf[7] = *(SendValue2_Pointer + 3);
//     Rx_Buf[8] = *SendValue3_Pointer;
//     Rx_Buf[9] = *(SendValue3_Pointer + 1);
//     Rx_Buf[10] = *(SendValue3_Pointer + 2);
//     Rx_Buf[11] = *(SendValue3_Pointer + 3);
//     Rx_Buf[12] = 0x00;
//     Rx_Buf[13] = 0x00;
//     Rx_Buf[14] = 0x80;
//     Rx_Buf[15] = 0x7F;
//
//     HAL_UART_Transmit_DMA(&huart7, Rx_Buf, sizeof(Rx_Buf));
// }


void USART_Vofa_Justfloat_Transmit(float SendValue1,float SendValue2,float SendValue3,float SendValue4,float SendValue5,float SendValue6, float SendValue7,float SendValue8,float SendValue9, float SendValue10,float SendValue11, float SendValue12, float SendValue13, float SendValue14, float SendValue15, float SendValue16, float SendValue17, float SendValue18, float SendValue19, float SendValue20, float SendValue21, float SendValue22, float SendValue23, float SendValue24, float SendValue25, float SendValue26, float SendValue27, float SendValue28, float SendValue29, float SendValue30, float SendValue31, float SendValue32, float SendValue33, float SendValue34, float SendValue35, float SendValue36, float SendValue37, float SendValue38, float SendValue39, float SendValue40, float SendValue41){

   __attribute__((section (".AXI_SRAM")))   static uint8_t Rx_Buf[168];

	uint8_t *SendValue1_Pointer, *SendValue2_Pointer, *SendValue3_Pointer, *SendValue4_Pointer, *SendValue5_Pointer, *SendValue6_Pointer, *SendValue7_Pointer, *SendValue8_Pointer, *SendValue9_Pointer, *SendValue10_Pointer, *SendValue11_Pointer, *SendValue12_Pointer, *SendValue13_Pointer, *SendValue14_Pointer, *SendValue15_Pointer, *SendValue16_Pointer, *SendValue17_Pointer, *SendValue18_Pointer, *SendValue19_Pointer, *SendValue20_Pointer, *SendValue21_Pointer, *SendValue22_Pointer, *SendValue23_Pointer, *SendValue24_Pointer, *SendValue25_Pointer, *SendValue26_Pointer, *SendValue27_Pointer, *SendValue28_Pointer, *SendValue29_Pointer, *SendValue30_Pointer, *SendValue31_Pointer, *SendValue32_Pointer, *SendValue33_Pointer, *SendValue34_Pointer, *SendValue35_Pointer, *SendValue36_Pointer, *SendValue37_Pointer, *SendValue38_Pointer, *SendValue39_Pointer, *SendValue40_Pointer, *SendValue41_Pointer;

	SendValue1_Pointer = (uint8_t *)&SendValue1;
	SendValue2_Pointer = (uint8_t *)&SendValue2;
 SendValue3_Pointer = (uint8_t *)&SendValue3;
	SendValue4_Pointer = (uint8_t *)&SendValue4;
	SendValue5_Pointer = (uint8_t *)&SendValue5;
	 SendValue6_Pointer = (uint8_t *)&SendValue6;
	 SendValue7_Pointer = (uint8_t *)&SendValue7;
	 SendValue8_Pointer = (uint8_t *)&SendValue8;
	 SendValue9_Pointer = (uint8_t *)&SendValue9;
	 SendValue10_Pointer = (uint8_t *)&SendValue10;
	 SendValue11_Pointer = (uint8_t *)&SendValue11;
	 SendValue12_Pointer = (uint8_t *)&SendValue12;
	 SendValue13_Pointer = (uint8_t *)&SendValue13;
	 SendValue14_Pointer = (uint8_t *)&SendValue14;
	 SendValue15_Pointer = (uint8_t *)&SendValue15;
	 SendValue16_Pointer = (uint8_t *)&SendValue16;
	 SendValue17_Pointer = (uint8_t *)&SendValue17;
	 SendValue18_Pointer = (uint8_t *)&SendValue18;
	 SendValue19_Pointer = (uint8_t *)&SendValue19;
	 SendValue20_Pointer = (uint8_t *)&SendValue20;
	 SendValue21_Pointer = (uint8_t *)&SendValue21;
	 SendValue22_Pointer = (uint8_t *)&SendValue22;
	 SendValue23_Pointer = (uint8_t *)&SendValue23;
	 SendValue24_Pointer = (uint8_t *)&SendValue24;
	 SendValue25_Pointer = (uint8_t *)&SendValue25;
	 SendValue26_Pointer = (uint8_t *)&SendValue26;
	 SendValue27_Pointer = (uint8_t *)&SendValue27;
	 SendValue28_Pointer = (uint8_t *)&SendValue28;
	 SendValue29_Pointer = (uint8_t *)&SendValue29;
	 SendValue30_Pointer = (uint8_t *)&SendValue30;
	 SendValue31_Pointer = (uint8_t *)&SendValue31;
	 SendValue32_Pointer = (uint8_t *)&SendValue32;
	 SendValue33_Pointer = (uint8_t *)&SendValue33;
	 SendValue34_Pointer = (uint8_t *)&SendValue34;
	 SendValue35_Pointer = (uint8_t *)&SendValue35;
	 SendValue36_Pointer = (uint8_t *)&SendValue36;
	 SendValue37_Pointer = (uint8_t *)&SendValue37;
	 SendValue38_Pointer = (uint8_t *)&SendValue38;
	 SendValue39_Pointer = (uint8_t *)&SendValue39;
	 SendValue40_Pointer = (uint8_t *)&SendValue40;
	 SendValue41_Pointer = (uint8_t *)&SendValue41;


    // 数据
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

	Rx_Buf[12] =  *SendValue4_Pointer;
	Rx_Buf[13] =  *(SendValue4_Pointer + 1);
	Rx_Buf[14] = *(SendValue4_Pointer + 2);
	Rx_Buf[15] = *(SendValue4_Pointer + 3);

	Rx_Buf[16] =  *SendValue5_Pointer;
	Rx_Buf[17] =  *(SendValue5_Pointer + 1);
	Rx_Buf[18] = *(SendValue5_Pointer + 2);
	Rx_Buf[19] = *(SendValue5_Pointer + 3);

	 Rx_Buf[20] =  *SendValue6_Pointer;
	 Rx_Buf[21] =  *(SendValue6_Pointer + 1);
	 Rx_Buf[22] = *(SendValue6_Pointer + 2);
	 Rx_Buf[23] = *(SendValue6_Pointer + 3);

	 Rx_Buf[24] =  *SendValue7_Pointer;
	 Rx_Buf[25] =  *(SendValue7_Pointer + 1);
	 Rx_Buf[26] = *(SendValue7_Pointer + 2);
	 Rx_Buf[27] = *(SendValue7_Pointer + 3);

	 Rx_Buf[28] =  *SendValue8_Pointer;
	 Rx_Buf[29] =  *(SendValue8_Pointer + 1);
	 Rx_Buf[30] = *(SendValue8_Pointer + 2);
	 Rx_Buf[31] = *(SendValue8_Pointer + 3);

	 Rx_Buf[32] =  *SendValue9_Pointer;
	 Rx_Buf[33] =  *(SendValue9_Pointer + 1);
	 Rx_Buf[34] = *(SendValue9_Pointer + 2);
	 Rx_Buf[35] = *(SendValue9_Pointer + 3);

	 Rx_Buf[36] =  *SendValue10_Pointer;
	 Rx_Buf[37] =  *(SendValue10_Pointer + 1);
	 Rx_Buf[38] = *(SendValue10_Pointer + 2);
	 Rx_Buf[39] = *(SendValue10_Pointer + 3);

	 Rx_Buf[40] =  *SendValue11_Pointer;
	 Rx_Buf[41] =  *(SendValue11_Pointer + 1);
	 Rx_Buf[42] = *(SendValue11_Pointer + 2);
	 Rx_Buf[43] = *(SendValue11_Pointer + 3);

	 Rx_Buf[44] =  *SendValue12_Pointer;
	 Rx_Buf[45] =  *(SendValue12_Pointer + 1);
	 Rx_Buf[46] = *(SendValue12_Pointer + 2);
	 Rx_Buf[47] = *(SendValue12_Pointer + 3);

	 Rx_Buf[48] =  *SendValue13_Pointer;
	 Rx_Buf[49] =  *(SendValue13_Pointer + 1);
	 Rx_Buf[50] = *(SendValue13_Pointer + 2);
	 Rx_Buf[51] = *(SendValue13_Pointer + 3);

	 Rx_Buf[52] =  *SendValue14_Pointer;
	 Rx_Buf[53] =  *(SendValue14_Pointer + 1);
	 Rx_Buf[54] = *(SendValue14_Pointer + 2);
	 Rx_Buf[55] = *(SendValue14_Pointer + 3);

	 Rx_Buf[56] =  *SendValue15_Pointer;
	 Rx_Buf[57] =  *(SendValue15_Pointer + 1);
	 Rx_Buf[58] = *(SendValue15_Pointer + 2);
	 Rx_Buf[59] = *(SendValue15_Pointer + 3);

	 Rx_Buf[60] =  *SendValue16_Pointer;
	 Rx_Buf[61] =  *(SendValue16_Pointer + 1);
	 Rx_Buf[62] = *(SendValue16_Pointer + 2);
	 Rx_Buf[63] = *(SendValue16_Pointer + 3);

	 Rx_Buf[64] =  *SendValue17_Pointer;
	 Rx_Buf[65] =  *(SendValue17_Pointer + 1);
	 Rx_Buf[66] = *(SendValue17_Pointer + 2);
	 Rx_Buf[67] = *(SendValue17_Pointer + 3);

	 Rx_Buf[68] =  *SendValue18_Pointer;
	 Rx_Buf[69] =  *(SendValue18_Pointer + 1);
	 Rx_Buf[70] = *(SendValue18_Pointer + 2);
	 Rx_Buf[71] = *(SendValue18_Pointer + 3);

	 Rx_Buf[72] =  *SendValue19_Pointer;
	 Rx_Buf[73] =  *(SendValue19_Pointer + 1);
	 Rx_Buf[74] = *(SendValue19_Pointer + 2);
	 Rx_Buf[75] = *(SendValue19_Pointer + 3);

	 Rx_Buf[76] =  *SendValue20_Pointer;
	 Rx_Buf[77] =  *(SendValue20_Pointer + 1);
	 Rx_Buf[78] = *(SendValue20_Pointer + 2);
	 Rx_Buf[79] = *(SendValue20_Pointer + 3);

	 Rx_Buf[80] =  *SendValue21_Pointer;
	 Rx_Buf[81] =  *(SendValue21_Pointer + 1);
	 Rx_Buf[82] = *(SendValue21_Pointer + 2);
	 Rx_Buf[83] = *(SendValue21_Pointer + 3);

	 Rx_Buf[84] =  *SendValue22_Pointer;
	 Rx_Buf[85] =  *(SendValue22_Pointer + 1);
	 Rx_Buf[86] = *(SendValue22_Pointer + 2);
	 Rx_Buf[87] = *(SendValue22_Pointer + 3);

	 Rx_Buf[88] =  *SendValue23_Pointer;
	 Rx_Buf[89] =  *(SendValue23_Pointer + 1);
	 Rx_Buf[90] = *(SendValue23_Pointer + 2);
	 Rx_Buf[91] = *(SendValue23_Pointer + 3);

	 Rx_Buf[92] =  *SendValue24_Pointer;
	 Rx_Buf[93] =  *(SendValue24_Pointer + 1);
	 Rx_Buf[94] = *(SendValue24_Pointer + 2);
	 Rx_Buf[95] = *(SendValue24_Pointer + 3);

	 Rx_Buf[96] =  *SendValue25_Pointer;
	 Rx_Buf[97] =  *(SendValue25_Pointer + 1);
	 Rx_Buf[98] = *(SendValue25_Pointer + 2);
	 Rx_Buf[99] = *(SendValue25_Pointer + 3);

	 Rx_Buf[100] =  *SendValue26_Pointer;
	 Rx_Buf[101] =  *(SendValue26_Pointer + 1);
	 Rx_Buf[102] = *(SendValue26_Pointer + 2);
	 Rx_Buf[103] = *(SendValue26_Pointer + 3);

	 Rx_Buf[104] =  *SendValue27_Pointer;
	 Rx_Buf[105] =  *(SendValue27_Pointer + 1);
	 Rx_Buf[106] = *(SendValue27_Pointer + 2);
	 Rx_Buf[107] = *(SendValue27_Pointer + 3);

	 Rx_Buf[108] =  *SendValue28_Pointer;
	 Rx_Buf[109] =  *(SendValue28_Pointer + 1);
	 Rx_Buf[110] = *(SendValue28_Pointer + 2);
	 Rx_Buf[111] = *(SendValue28_Pointer + 3);

	 Rx_Buf[112] =  *SendValue29_Pointer;
	 Rx_Buf[113] =  *(SendValue29_Pointer + 1);
	 Rx_Buf[114] = *(SendValue29_Pointer + 2);
	 Rx_Buf[115] = *(SendValue29_Pointer + 3);

	 Rx_Buf[116] =  *SendValue30_Pointer;
	 Rx_Buf[117] =  *(SendValue30_Pointer + 1);
	 Rx_Buf[118] = *(SendValue30_Pointer + 2);
	 Rx_Buf[119] = *(SendValue30_Pointer + 3);

	 Rx_Buf[120] =  *SendValue31_Pointer;
	 Rx_Buf[121] =  *(SendValue31_Pointer + 1);
	 Rx_Buf[122] = *(SendValue31_Pointer + 2);
	 Rx_Buf[123] = *(SendValue31_Pointer + 3);

	 Rx_Buf[124] =  *SendValue32_Pointer;
	 Rx_Buf[125] =  *(SendValue32_Pointer + 1);
	 Rx_Buf[126] = *(SendValue32_Pointer + 2);
	 Rx_Buf[127] = *(SendValue32_Pointer + 3);

	 Rx_Buf[128] =  *SendValue33_Pointer;
	 Rx_Buf[129] =  *(SendValue33_Pointer + 1);
	 Rx_Buf[130] = *(SendValue33_Pointer + 2);
	 Rx_Buf[131] = *(SendValue33_Pointer + 3);

	 Rx_Buf[132] =  *SendValue34_Pointer;
	 Rx_Buf[133] =  *(SendValue34_Pointer + 1);
	 Rx_Buf[134] = *(SendValue34_Pointer + 2);
	 Rx_Buf[135] = *(SendValue34_Pointer + 3);

	Rx_Buf[136] =  *SendValue35_Pointer;
	Rx_Buf[137] =  *(SendValue35_Pointer + 1);
	Rx_Buf[138] = *(SendValue35_Pointer + 2);
	Rx_Buf[139] = *(SendValue35_Pointer + 3);

	 Rx_Buf[140] =  *SendValue36_Pointer;
	 Rx_Buf[141] =  *(SendValue36_Pointer + 1);
	 Rx_Buf[142] = *(SendValue36_Pointer + 2);
	 Rx_Buf[143] = *(SendValue36_Pointer + 3);

	 Rx_Buf[144] =  *SendValue37_Pointer;
	 Rx_Buf[145] =  *(SendValue37_Pointer + 1);
	 Rx_Buf[146] = *(SendValue37_Pointer + 2);
	 Rx_Buf[147] = *(SendValue37_Pointer + 3);

	 Rx_Buf[148] =  *SendValue38_Pointer;
	 Rx_Buf[149] =  *(SendValue38_Pointer + 1);
	 Rx_Buf[150] = *(SendValue38_Pointer + 2);
	 Rx_Buf[151] = *(SendValue38_Pointer + 3);

	 Rx_Buf[152] =  *SendValue39_Pointer;
	 Rx_Buf[153] =  *(SendValue39_Pointer + 1);
	 Rx_Buf[154] = *(SendValue39_Pointer + 2);
	 Rx_Buf[155] = *(SendValue39_Pointer + 3);

	 Rx_Buf[156] =  *SendValue40_Pointer;
	 Rx_Buf[157] =  *(SendValue40_Pointer + 1);
	 Rx_Buf[158] = *(SendValue40_Pointer + 2);
	 Rx_Buf[159] = *(SendValue40_Pointer + 3);

	 Rx_Buf[160] =  *SendValue41_Pointer;
	 Rx_Buf[161] =  *(SendValue41_Pointer + 1);
	 Rx_Buf[162] = *(SendValue41_Pointer + 2);
	 Rx_Buf[163] = *(SendValue41_Pointer + 3);

    // 帧尾
    Rx_Buf[164] =  0x00;
    Rx_Buf[165] =  0x00;
    Rx_Buf[166] =  0x80;
    Rx_Buf[167] =  0x7F;

	HAL_UART_Transmit_DMA(&huart7,Rx_Buf,sizeof(Rx_Buf));

}
#include "DJI_motor.h"
#include "bsp_can.h"
#include "user_lib.h"

/**
 * @brief DJI电机解码
 * @param[in] motor  电机结构体指针
 * @param[in] data   接收到的数据的指针
 */
void DJI_Info_Update(DJI_Motor_t *motor, uint8_t *data) {
    /* 转子机械角度 */
    motor->ecd = (uint16_t)(data[0]<<8 | data[1]);
    /* 转子转速 */
    motor->speed_rpm = (int16_t)(data[2]<<8 | data[3]);
    /* 实际扭矩电流 */
    motor->given_current = (int16_t)(data[4]<<8 | data[5]);
    /* 电机温度 */
    motor->temperate = data[6];
}

void DJI_Current_Set(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4) {

    WheelTxFrame.Data[0] = motor1 >> 8;
    WheelTxFrame.Data[1] = motor1;
    WheelTxFrame.Data[2] = motor2 >> 8;
    WheelTxFrame.Data[3] = motor2;
    WheelTxFrame.Data[4] = motor3 >> 8;
    WheelTxFrame.Data[5] = motor3;
    WheelTxFrame.Data[6] = motor4 >> 8;
    WheelTxFrame.Data[7] = motor4;

    CAN_SendWheelData(&WheelTxFrame);

}
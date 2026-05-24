#include "DJI_motor.h"
#include "bsp_can.h"
#include "user_lib.h"


float DJI_SpeedSmoothFilter(float last_speed, float raw_speed, float smooth_coef) {
    if (smooth_coef > 1.0f) {
        smooth_coef = 1.0f;
    } else if (smooth_coef < 0.0f) {
        smooth_coef = 0.0f;
    }

    return (1.0f - smooth_coef) * last_speed + smooth_coef * raw_speed;
}
/**
 * @brief DJI电机解码
 * @param[in] motor  电机结构体指针
 * @param[in] data   接收到的数据的指针
 */
void DJI_Info_Update(DJI_Motor_t *motor, uint8_t *data)
{
    /* 转子机械角度 */
    motor->ecd = (uint16_t) (data[0] << 8 | data[1]);

    /* 转子转速 */
    motor->speed_rpm = (int16_t) (data[2] << 8 | data[3]);

    // 轻度去毛刺
    float raw_speed_aps = (float) motor->speed_rpm * RPM_TO_ANGLE_PER_SEC;

    if (motor->speed_filter_inited == 0)
    {
        motor->speed_aps = raw_speed_aps;
        motor->speed_filter_inited = 1;
    }
    else
    {
        motor->speed_aps = DJI_SpeedSmoothFilter(motor->speed_aps,
                                                 raw_speed_aps,
                                                 DJI_SPEED_SMOOTH_COEF);
    }

    /* 实际扭矩电流 */
    motor->given_current = (int16_t) (data[4] << 8 | data[5]);
    /* 电机温度 */
    motor->temperate = data[6];
}

void DJI_Current_Set(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4) {

    Board_Yaw_Wheel_TxFrame.Header.Identifier = 0x1FF;

    Board_Yaw_Wheel_TxFrame.Data[0] = motor1 >> 8;
    Board_Yaw_Wheel_TxFrame.Data[1] = motor1;
    Board_Yaw_Wheel_TxFrame.Data[2] = motor2 >> 8;
    Board_Yaw_Wheel_TxFrame.Data[3] = motor2;
    Board_Yaw_Wheel_TxFrame.Data[4] = motor3 >> 8;
    Board_Yaw_Wheel_TxFrame.Data[5] = motor3;
    Board_Yaw_Wheel_TxFrame.Data[6] = motor4 >> 8;
    Board_Yaw_Wheel_TxFrame.Data[7] = motor4;

    HAL_FDCAN_AddMessageToTxFifoQ(Board_Yaw_Wheel_TxFrame.hcan, &Board_Yaw_Wheel_TxFrame.Header,
                                  Board_Yaw_Wheel_TxFrame.Data);

}
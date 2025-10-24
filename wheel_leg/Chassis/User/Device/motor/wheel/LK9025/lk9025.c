#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_can.h>
#include "lk9025.h"
#include "user_lib.h"
#include "bsp_can.h"

#include "robot_def.h"

Lk9025 *lk_motors[2];
uint8_t lk_motors_len = 0;

/** 这两个函数是为了将wheel.c定义的wheel[2]和lk9025.c定义的lk_motors[2]绑定在一起 **/
static void lk9025_register(Lk9025* motor) {
    lk_motors[lk_motors_len] = motor;
    ++lk_motors_len;
}

void lk9025_init(Lk9025 *motor, uint32_t device_id) {
    motor->id = device_id;

    motor->torque = 0;
    motor->angular_vel = 0;

    lk9025_register(motor);
}
/****************************************************************************/

/** 单电机扭矩闭环 **/
void lk9025_torque_set(Lk9025 *motor, float motor_torque)
{
    WheelTxFrame.Header.StdId = motor->id;

    float motor_current = motor_torque / LK_TORQUE_CONSTANT;
    int16_t motor_data = motor_current * LK_CURRENT_2_DATA;

    WheelTxFrame.Data[0] = 0xA1;
    WheelTxFrame.Data[1] = 0;
    WheelTxFrame.Data[2] = 0;
    WheelTxFrame.Data[3] = 0;
    WheelTxFrame.Data[4] = *(uint8_t *) (&motor_data);
    WheelTxFrame.Data[5] = *((uint8_t *) (&motor_data) + 1);
    WheelTxFrame.Data[6] = 0;
    WheelTxFrame.Data[7] = 0;

    CAN_SendWheelData(&WheelTxFrame);
}

/** 一拖四模式 -- 多电机转矩闭环 **/
void lk9025_multi_torque_set(float motor1_torque, float motor2_torque) {

    float motor1_current, motor2_current;
    motor1_current = motor1_torque / LK_TORQUE_CONSTANT;
    motor2_current = motor2_torque / LK_TORQUE_CONSTANT;

    int16_t motor1_data, motor2_data;
    motor1_data = motor1_current * LK_CURRENT_2_DATA;
    motor2_data = motor2_current * LK_CURRENT_2_DATA;

    WheelTxFrame.Data[0] = *(uint8_t *)(&motor1_data);
    WheelTxFrame.Data[1] = *((uint8_t *)(&motor1_data) + 1);
    WheelTxFrame.Data[2] = *(uint8_t *)(&motor2_data);
    WheelTxFrame.Data[3] = *((uint8_t *)(&motor2_data) + 1);
    WheelTxFrame.Data[4] = 0;
    WheelTxFrame.Data[5] = 0;
    WheelTxFrame.Data[6] = 0;
    WheelTxFrame.Data[7] = 0;

    CAN_SendWheelData(&WheelTxFrame);
}

/** 轮毂电机反馈解析 **/
void lk9025_info_update(Lk9025* motor, uint8_t data[])
{
    int16_t iq_int = (int16_t) ((data)[3] << 8 | (data)[2]);

    // °/s
    int16_t speed_int = (int16_t) ((data)[5] << 8 | (data)[4]);

    /** 力矩 Nm **/
    motor->torque = (iq_int / LK_CURRENT_2_DATA) * LK_TORQUE_CONSTANT;

    /** 角速度 rad/s **/
    motor->angular_vel = speed_int * DEGREE_TO_RAD;


}

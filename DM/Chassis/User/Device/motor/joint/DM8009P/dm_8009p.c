#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_can.h>

#include "dm_8009p.h"
#include "bsp_can.h"

Dm8009P *dm_8009p[4];
uint8_t dm_8009p_len = 0;

static int float_to_uint(float x, float x_min, float x_max, int bits) {
    /// Converts a float to an unsigned int, given range and number of bits///
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x - offset) * ((float) ((1 << bits) - 1)) / span);
}

static float uint_to_float(int x_int, float x_min, float x_max, int bits) {
/// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float) x_int) * span / ((float) ((1 << bits) - 1)) + offset;
}

/** 这两个函数是为了将joint.c定义的joint[4]和dm8009.c定义的dm_motors[4]绑定在一起 **/
static void dm8009p_register(Dm8009P *motor) {
    dm_8009p[dm_8009p_len] = motor;
    ++dm_8009p_len;
}

void dm8009p_init(Dm8009P *motor, uint32_t device_id) {
    motor->id = device_id;

    dm8009p_register(motor);
}
/****************************************************************************/

/** 使能电机 **/
void set_dm8009p_enable(Dm8009P* motor){

    JointTxFrame.Header.StdId = motor->id;

    JointTxFrame.Data[0] = 0xFF;
    JointTxFrame.Data[1] = 0xFF;
    JointTxFrame.Data[2] = 0xFF;
    JointTxFrame.Data[3] = 0xFF;
    JointTxFrame.Data[4] = 0xFF;
    JointTxFrame.Data[5] = 0xFF;
    JointTxFrame.Data[6] = 0xFF;
    JointTxFrame.Data[7] = 0xFC;

    CAN_SendJointData(&JointTxFrame);
}

/** 失能电机 **/
void set_dm8009p_disable(Dm8009P* motor){

    JointTxFrame.Header.StdId = motor->id;

    JointTxFrame.Data[0] = 0xFF;
    JointTxFrame.Data[1] = 0xFF;
    JointTxFrame.Data[2] = 0xFF;
    JointTxFrame.Data[3] = 0xFF;
    JointTxFrame.Data[4] = 0xFF;
    JointTxFrame.Data[5] = 0xFF;
    JointTxFrame.Data[6] = 0xFF;
    JointTxFrame.Data[7] = 0xFD;

    CAN_SendJointData(&JointTxFrame);
}

/** 位置速度模式模式 **/
void set_dm8009p_pos_speed(Dm8009P* motor,
                          float pos_rad,
                          float speed_rad_per_s) {

    JointTxFrame.Header.StdId = motor->id + 0x100;

    uint8_t *posbuf = (uint8_t *) &pos_rad;
    uint8_t *speedbuf = (uint8_t *) &speed_rad_per_s;

    JointTxFrame.Data[0] = *posbuf;
    JointTxFrame.Data[1] = *(posbuf + 1);
    JointTxFrame.Data[2] = *(posbuf + 2);
    JointTxFrame.Data[3] = *(posbuf + 3);
    JointTxFrame.Data[4] = *speedbuf;
    JointTxFrame.Data[5] = *(speedbuf + 1);
    JointTxFrame.Data[6] = *(speedbuf + 2);
    JointTxFrame.Data[7] = *(speedbuf + 3);

    CAN_SendJointData(&JointTxFrame);
}

/** 单电机MIT模式 **/
void set_dm8009p_MIT(Dm8009P* motor,
                    float pos,
                    float speed,
                    float kp,
                    float kd,
                    float torque) {

    JointTxFrame.Header.StdId = motor->id;

    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    pos_tmp = float_to_uint(pos, DM8009P_P_MIN, DM8009P_P_MAX, 16); // -12.5 ~ 12.5
    vel_tmp = float_to_uint(speed, DM8009P_V_MIN, DM8009P_V_MAX, 12); // -45 ~ 45
    kp_tmp = float_to_uint(kp, DM8009P_KP_MIN, DM8009P_KP_MAX, 12); // 0 ~ 500
    kd_tmp = float_to_uint(kd, DM8009P_KD_MIN, DM8009P_KD_MAX, 12); // 0 ~ 5
    tor_tmp = float_to_uint(torque, DM8009P_T_MIN, DM8009P_T_MAX, 12); // -50 ~ 50

    JointTxFrame.Data[0] = (pos_tmp >> 8);
    JointTxFrame.Data[1] = pos_tmp;
    JointTxFrame.Data[2] = (vel_tmp >> 4);
    JointTxFrame.Data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    JointTxFrame.Data[4] = kp_tmp;
    JointTxFrame.Data[5] = (kd_tmp >> 4);
    JointTxFrame.Data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    JointTxFrame.Data[7] = tor_tmp;

    CAN_SendJointData(&JointTxFrame);
}

/** 关节电机反馈解析 **/
void dm8009p_info_update(Dm8009P* motor, uint8_t data[])
{
    int pos_int = (data[1] << 8) | data[2];
    int speed_int = (data[3] << 4) | (data[4] >> 4);
    int torque_int = (data[4] & 0xF) << 8 | data[5];

    motor->pos_r = uint_to_float(pos_int, DM8009P_P_MIN, DM8009P_P_MAX, 16);
    motor->angular_vel = uint_to_float(speed_int, DM8009P_V_MIN, DM8009P_V_MAX, 12);
    motor->torque = uint_to_float(torque_int, DM8009P_T_MIN, DM8009P_T_MAX, 12);
}


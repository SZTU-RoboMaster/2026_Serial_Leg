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

/** 使能左侧电机 **/
void set_left_dm8009p_enable(Dm8009P *left_motor) {

    LeftJointTxFrame.Header.Identifier = left_motor->id;

    LeftJointTxFrame.Data[0] = 0xFF;
    LeftJointTxFrame.Data[1] = 0xFF;
    LeftJointTxFrame.Data[2] = 0xFF;
    LeftJointTxFrame.Data[3] = 0xFF;
    LeftJointTxFrame.Data[4] = 0xFF;
    LeftJointTxFrame.Data[5] = 0xFF;
    LeftJointTxFrame.Data[6] = 0xFF;
    LeftJointTxFrame.Data[7] = 0xFC;

    HAL_FDCAN_AddMessageToTxFifoQ(LeftJointTxFrame.hcan, &LeftJointTxFrame.Header, LeftJointTxFrame.Data);
}

/** 使能右侧电机 **/
void set_right_dm8009p_enable(Dm8009P *right_motor) {

    RightJointTxFrame.Header.Identifier = right_motor->id;

    RightJointTxFrame.Data[0] = 0xFF;
    RightJointTxFrame.Data[1] = 0xFF;
    RightJointTxFrame.Data[2] = 0xFF;
    RightJointTxFrame.Data[3] = 0xFF;
    RightJointTxFrame.Data[4] = 0xFF;
    RightJointTxFrame.Data[5] = 0xFF;
    RightJointTxFrame.Data[6] = 0xFF;
    RightJointTxFrame.Data[7] = 0xFC;

    HAL_FDCAN_AddMessageToTxFifoQ(RightJointTxFrame.hcan, &RightJointTxFrame.Header, RightJointTxFrame.Data);
}


/** 左侧电机-位置速度模式模式 **/
void set_left_dm8009p_pos_speed(Dm8009P *left_motor,
                                float pos_rad,
                                float speed_rad_per_s) {

    LeftJointTxFrame.Header.Identifier = left_motor->id + 0x100;

    uint8_t *posbuf = (uint8_t *) &pos_rad;
    uint8_t *speedbuf = (uint8_t *) &speed_rad_per_s;

    LeftJointTxFrame.Data[0] = *posbuf;
    LeftJointTxFrame.Data[1] = *(posbuf + 1);
    LeftJointTxFrame.Data[2] = *(posbuf + 2);
    LeftJointTxFrame.Data[3] = *(posbuf + 3);
    LeftJointTxFrame.Data[4] = *speedbuf;
    LeftJointTxFrame.Data[5] = *(speedbuf + 1);
    LeftJointTxFrame.Data[6] = *(speedbuf + 2);
    LeftJointTxFrame.Data[7] = *(speedbuf + 3);

    HAL_FDCAN_AddMessageToTxFifoQ(LeftJointTxFrame.hcan, &LeftJointTxFrame.Header, LeftJointTxFrame.Data);
}

/** 右侧电机-位置速度模式模式 **/
void set_right_dm8009p_pos_speed(Dm8009P *right_motor,
                                 float pos_rad,
                                 float speed_rad_per_s) {

    RightJointTxFrame.Header.Identifier = right_motor->id + 0x100;

    uint8_t *posbuf = (uint8_t *) &pos_rad;
    uint8_t *speedbuf = (uint8_t *) &speed_rad_per_s;

    RightJointTxFrame.Data[0] = *posbuf;
    RightJointTxFrame.Data[1] = *(posbuf + 1);
    RightJointTxFrame.Data[2] = *(posbuf + 2);
    RightJointTxFrame.Data[3] = *(posbuf + 3);
    RightJointTxFrame.Data[4] = *speedbuf;
    RightJointTxFrame.Data[5] = *(speedbuf + 1);
    RightJointTxFrame.Data[6] = *(speedbuf + 2);
    RightJointTxFrame.Data[7] = *(speedbuf + 3);

    HAL_FDCAN_AddMessageToTxFifoQ(RightJointTxFrame.hcan, &RightJointTxFrame.Header, RightJointTxFrame.Data);
}

/** 左侧电机-MIT模式 **/
void set_left_dm8009p_MIT(Dm8009P *left_motor,
                          float pos,
                          float speed,
                          float kp,
                          float kd,
                          float torque) {

    LeftJointTxFrame.Header.Identifier = left_motor->id;

    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    pos_tmp = float_to_uint(pos, DM8009P_P_MIN, DM8009P_P_MAX, 16); // -12.5 ~ 12.5
    vel_tmp = float_to_uint(speed, DM8009P_V_MIN, DM8009P_V_MAX, 12); // -45 ~ 45
    kp_tmp = float_to_uint(kp, DM8009P_KP_MIN, DM8009P_KP_MAX, 12); // 0 ~ 500
    kd_tmp = float_to_uint(kd, DM8009P_KD_MIN, DM8009P_KD_MAX, 12); // 0 ~ 5
    tor_tmp = float_to_uint(torque, DM8009P_T_MIN, DM8009P_T_MAX, 12); // -50 ~ 50

    LeftJointTxFrame.Data[0] = (pos_tmp >> 8);
    LeftJointTxFrame.Data[1] = pos_tmp;
    LeftJointTxFrame.Data[2] = (vel_tmp >> 4);
    LeftJointTxFrame.Data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    LeftJointTxFrame.Data[4] = kp_tmp;
    LeftJointTxFrame.Data[5] = (kd_tmp >> 4);
    LeftJointTxFrame.Data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    LeftJointTxFrame.Data[7] = tor_tmp;

    HAL_FDCAN_AddMessageToTxFifoQ(LeftJointTxFrame.hcan, &LeftJointTxFrame.Header, LeftJointTxFrame.Data);
}

/** 右侧电机-MIT模式 **/
void set_right_dm8009p_MIT(Dm8009P *right_motor,
                           float pos,
                           float speed,
                           float kp,
                           float kd,
                           float torque) {

    RightJointTxFrame.Header.Identifier = right_motor->id;

    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    pos_tmp = float_to_uint(pos, DM8009P_P_MIN, DM8009P_P_MAX, 16); // -12.5 ~ 12.5
    vel_tmp = float_to_uint(speed, DM8009P_V_MIN, DM8009P_V_MAX, 12); // -45 ~ 45
    kp_tmp = float_to_uint(kp, DM8009P_KP_MIN, DM8009P_KP_MAX, 12); // 0 ~ 500
    kd_tmp = float_to_uint(kd, DM8009P_KD_MIN, DM8009P_KD_MAX, 12); // 0 ~ 5
    tor_tmp = float_to_uint(torque, DM8009P_T_MIN, DM8009P_T_MAX, 12); // -50 ~ 50

    RightJointTxFrame.Data[0] = (pos_tmp >> 8);
    RightJointTxFrame.Data[1] = pos_tmp;
    RightJointTxFrame.Data[2] = (vel_tmp >> 4);
    RightJointTxFrame.Data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    RightJointTxFrame.Data[4] = kp_tmp;
    RightJointTxFrame.Data[5] = (kd_tmp >> 4);
    RightJointTxFrame.Data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    RightJointTxFrame.Data[7] = tor_tmp;

    HAL_FDCAN_AddMessageToTxFifoQ(RightJointTxFrame.hcan, &RightJointTxFrame.Header, RightJointTxFrame.Data);
}

/** 关节电机反馈解析 **/
void dm8009p_info_update(Dm8009P *motor, uint8_t data[]) {
    int pos_int = (data[1] << 8) | data[2];
    int speed_int = (data[3] << 4) | (data[4] >> 4);
    int torque_int = (data[4] & 0xF) << 8 | data[5];

    motor->pos_r = uint_to_float(pos_int, DM8009P_P_MIN, DM8009P_P_MAX, 16);
    motor->angular_vel = uint_to_float(speed_int, DM8009P_V_MIN, DM8009P_V_MAX, 12);
    motor->torque = uint_to_float(torque_int, DM8009P_T_MIN, DM8009P_T_MAX, 12);
}


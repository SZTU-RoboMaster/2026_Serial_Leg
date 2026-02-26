#ifndef DM_8009P_H
#define DM_8009P_H

#include "stdint-gcc.h"
#include "can_device.h"

/********************   电机参数限制   *****************************/

#define DM8009P_P_MIN -12.5f
#define DM8009P_P_MAX 12.5f
#define DM8009P_V_MIN -45.0f
#define DM8009P_V_MAX 45.0f
#define DM8009P_KP_MIN 0.0f
#define DM8009P_KP_MAX 500.0f
#define DM8009P_KD_MIN 0.0f
#define DM8009P_KD_MAX 5.0f
#define DM8009P_T_MIN -50.0f
#define DM8009P_T_MAX 50.0f


typedef struct {
    uint32_t id;
    /** 绝对位置 **/
    float pos_r;

    /** 关节电机角速度 **/
    float angular_vel;

    /** 关节电机反馈力矩 **/
    float torque;
} Dm8009P;

/** 初始化电机ID **/
void dm8009p_init(Dm8009P *motor, uint32_t device_id);

/** 使能电机 **/
void set_left_dm8009p_enable(Dm8009P *left_motor);

void set_right_dm8009p_enable(Dm8009P *right_motor);


/** 位置速度模式模式 **/
void set_left_dm8009p_pos_speed(Dm8009P *left_motor,
                                float pos_rad,
                                float speed_rad_per_s);

void set_right_dm8009p_pos_speed(Dm8009P *right_motor,
                                 float pos_rad,
                                 float speed_rad_per_s);

/** 单电机MIT模式 **/
void set_left_dm8009p_MIT(Dm8009P *left_motor,
                          float pos,
                          float speed,
                          float kp,
                          float kd,
                          float torque);

void set_right_dm8009p_MIT(Dm8009P *right_motor,
                           float pos,
                           float speed,
                           float kp,
                           float kd,
                           float torque);

/** 关节电机反馈解析 **/
void dm8009p_info_update(Dm8009P *motor, uint8_t data[]);


#endif

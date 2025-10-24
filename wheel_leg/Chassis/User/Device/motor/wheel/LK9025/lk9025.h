#ifndef LK_9025_H
#define LK_9025_H

#include <stdint-gcc.h>
#include "can_device.h"
/********************  一拖四相关参数  *****************************/
/** 力矩常数(1A对应的力矩 单位 Nm) **/
#define LK_TORQUE_CONSTANT 0.32f

/** 1A电流对应的数值 value / A **/
#define LK_CURRENT_2_DATA 62.5f

typedef struct{

    uint32_t id;

    /** 角速度 **/
    float angular_vel;

    /** 力矩 **/
    float torque;
} Lk9025;

void lk9025_init(Lk9025 *motor, uint32_t device_id);

void lk9025_torque_set(Lk9025 *motor, float motor_torque);

/** 多电机转矩闭环 **/
void lk9025_multi_torque_set(float motor1_torque, float motor2_torque);

/** 轮毂电机反馈解析 **/
void lk9025_info_update(Lk9025 *motor, uint8_t data[]);

#endif

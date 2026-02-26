#ifndef WHEEL_H
#define WHEEL_H

#include "can_device.h"
#include "DJI_motor.h"

enum WheelMotorIndex{
    L = 0,
    R = 1,
};

/** 轮毂电机初始化 **/
void wheel_init(void);

/** 返回轮毂电机指针 **/
DJI_Motor_t *get_wheel_motors();

extern DJI_Motor_t wheel[2];

#endif

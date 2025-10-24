#ifndef WHEEL_H
#define WHEEL_H

#include "can_device.h"
#include "lk9025.h"

enum WheelMotorIndex{
    L = 0,
    R = 1,
};

/** 轮毂电机初始化 **/
void wheel_init(void);

/** 返回轮毂电机指针 **/
Lk9025 *get_wheel_motors();

extern Lk9025 wheel[2];

#endif

#ifndef JOINT_H
#define JOINT_H

#include "can_device.h"
#include "dm_8009.h"

// 10°对应0.174rad
#define RESET_THRESHOLD 0.522f

enum JointMotorIndex{
    LF=0,
    LB=1,
    RF=2,
    RB=3,
};

/** 初始化关节电机ID **/
void joint_init(void);

/** 使能关节电机 **/
void joint_enable(void);

/** 返回关节电机指针 **/
Dm8009* get_joint_motors(void);

extern Dm8009 joint[4];

extern float Kd;
extern float vel;

#endif

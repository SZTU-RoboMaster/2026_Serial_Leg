#ifndef JOINT_H
#define JOINT_H

#include "can_device.h"
#include "dm_8009p.h"

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
Dm8009P* get_joint_motors(void);

extern Dm8009P joint[4];

#endif

#ifndef JOINT_H
#define JOINT_H

#include "can_device.h"
#include "dm_8009p.h"

enum JointMotorIndex
{
    LF=0,
    LB=1,
    RF=2,
    RB=3,
};

void joint_init(void);
void joint_enable(void);
void joint_disable(void);
Dm8009P* get_joint_motors(void);

extern Dm8009P joint[4];

#endif

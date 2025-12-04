#include "cmsis_os.h"
#include "robot_def.h"
#include "joint.h"
#include "bsp_dwt.h"

Dm8009P joint[4];

/** 初始化关节电机ID **/
void joint_init(void) {
    dm8009p_init(&joint[LF], JOINT_LF_SEND);
    dm8009p_init(&joint[LB], JOINT_LB_SEND);
    dm8009p_init(&joint[RF], JOINT_RF_SEND);
    dm8009p_init(&joint[RB], JOINT_RB_SEND);
}

/** 使能关节电机 **/
void joint_enable(void) {
    for(int i = 0; i < 10; i ++)
    {
        set_dm8009p_enable(&joint[LF]);
        osDelay(1);
    }
    for(int i = 0; i < 10; i ++)
    {
        set_dm8009p_enable(&joint[LB]);
        osDelay(1);
    }
    for(int i = 0; i < 10; i ++)
    {
        set_dm8009p_enable(&joint[RF]);
        osDelay(1);
    }
    for(int i = 0; i < 10; i ++)
    {
        set_dm8009p_enable(&joint[RB]);
        osDelay(1);
    }
}


/** 返回关节电机指针 **/
Dm8009P* get_joint_motors(void)
{
    return joint;
}

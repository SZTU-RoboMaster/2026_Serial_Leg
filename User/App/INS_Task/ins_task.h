#ifndef _INS_TASK_H
#define _INS_TASK_H
#include <QuaternionEKF.h>


typedef struct {
    float q[4]; // 四元数估计值

    float Accel[3]; // 加速度
    float MotionAccel_b[3]; // 机体坐标加速度
    float MotionAccel_n[3]; // 绝对系加速度

    float AccelLPF; // 加速度低通滤波系数

    // 加速度在绝对系的向量表示
    float xn[3];
    float yn[3];
    float zn[3];

} INS_t;

void INS_Init(void);
void Body_Accel_To_Earth(void);
extern INS_t INS;
extern QEKF_INS_t QEKF_INS;
extern uint32_t INS_DWT_Count;

#endif

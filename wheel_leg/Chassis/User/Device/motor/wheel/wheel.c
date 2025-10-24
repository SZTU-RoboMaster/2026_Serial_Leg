#include "wheel.h"


Lk9025 wheel[2];

/** 轮毂电机初始化 **/
void wheel_init(void)
{
    lk9025_init(&wheel[L], WHEEL_L_SEND);
    lk9025_init(&wheel[R], WHEEL_R_SEND);
}

/** 返回轮毂电机指针 **/
Lk9025* get_wheel_motors(void){
    return wheel;
}

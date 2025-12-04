#include "wheel.h"

DJI_Motor_t wheel[2];

/** 轮毂电机初始化 **/
void wheel_init(void)
{

}

/** 返回轮毂电机指针 **/
DJI_Motor_t* get_wheel_motors(void){
    return wheel;
}

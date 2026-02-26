#include "wheel.h"

DJI_Motor_t wheel[2];

/* =========================== 轮毂电机初始化 ============================= */

    /* iss：补 */

    void wheel_init(void)
    {

    }

/* =========================== 返回指针 ============================= */

    DJI_Motor_t* get_wheel_motors(void)
    {
        return wheel;
    }

#ifndef _BOARD_COMMUNICATION_TASK_H
#define _BOARD_COMMUNICATION_TASK_H

#include <stdint-gcc.h>
#include <stdbool.h>

union I16{
    uint8_t data[2];
    int16_t value;
};

typedef struct
{
    union I16 vx_channel; // 前后
    union I16 leg_channel; // 腿长

    char sr;

    float yaw_relative_angle; // yaw与底盘正方向的相对角度



}Gimbal_Unpack_Data;

void Gimbal_Data_Unpack(const uint8_t *rx_data);

#endif

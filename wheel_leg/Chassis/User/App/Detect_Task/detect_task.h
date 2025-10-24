#ifndef _DETECT_TASK_H
#define _DETECT_TASK_H

#include "stm32f4xx_hal.h"
#include <stdint-gcc.h>

#define ONLINE 1
#define OFFLINE 0

typedef struct {

    uint32_t last_online_time;
    uint8_t status;
    uint32_t offline_threshold;
    uint8_t warning_level;

} detect_device_t;

typedef enum {
    DETECT_CAP = 0,

    CHASSIS_JOINT_LF,
    CHASSIS_JOINT_LB,
    CHASSIS_JOINT_RF,
    CHASSIS_JOINT_RB,
    CHASSIS_WHEEL_L,
    CHASSIS_WHEEL_R,
    DETECT_DEVICE_LIST_LEN,
}detect_device_index;

void detect_handle(uint8_t index);

#endif

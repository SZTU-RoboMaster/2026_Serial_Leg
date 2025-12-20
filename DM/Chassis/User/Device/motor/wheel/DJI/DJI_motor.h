#ifndef _DJI_MOTOR_H
#define _DJI_MOTOR_H

#include <stdint-gcc.h>

#define RATIO 14.88 // 港科减速箱减速比
#define TORQUE_CONSTANT_3508 0.2325f  // Nm/A 港科减速箱

#define DATA_PER_A (16384 / 20)

//电机的数据
typedef struct {
    /* 实际电机反馈数据 */
    uint16_t ecd;           //转子机械角度, 电机编码器计数值
    int16_t speed_rpm;      //转子转速, 电机转速（每分钟转数，RPM）
    int16_t given_current;  //实际扭矩电流
    uint8_t temperate;      //电机温度

} DJI_Motor_t;

void DJI_Info_Update(DJI_Motor_t *motor, uint8_t *data);

void DJI_Current_Set(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);


#endif


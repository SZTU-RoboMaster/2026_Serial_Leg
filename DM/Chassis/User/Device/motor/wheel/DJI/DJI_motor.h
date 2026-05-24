#ifndef _DJI_MOTOR_H
#define _DJI_MOTOR_H

#include <stdint-gcc.h>

#define RATIO 14.88 // 港科减速箱减速比
#define TORQUE_CONSTANT_3508 0.2325f  // Nm/A 港科减速箱

#define DJI_SPEED_SMOOTH_COEF 0.85f  // 反馈速度平滑系数
#define RPM_TO_ANGLE_PER_SEC 6.0f    // rpm -> °/s

#define DATA_PER_A (16384 / 20)

//电机的数据
typedef struct {
    /* 实际电机反馈数据 */
    uint16_t ecd;           //转子机械角度, 电机编码器计数值
    int16_t speed_rpm;      //转子转速, 电机转速（每分钟转数，RPM）
    float speed_aps;        //平滑滤波后的转子角速度（deg/s）
    uint8_t speed_filter_inited;
    int16_t given_current;  //实际扭矩电流
    uint8_t temperate;      //电机温度

} DJI_Motor_t;

float DJI_SpeedSmoothFilter(float last_speed, float raw_speed, float smooth_coef);

void DJI_Info_Update(DJI_Motor_t *motor, uint8_t *data);

void DJI_Current_Set(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);


#endif


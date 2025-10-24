#ifndef _DJI_MOTOR_H
#define _DJI_MOTOR_H

#include "bsp_can.h"

// C620/C610 id=1~4 (0x201~0x204)
#define CAN_DJI_MOTOR_0x200_ID 0x200

// C620/C610 id=5~8 (0x205~0x208)
// GM6020 id=1~4 (0x205~0x208)
#define CAN_DJI_MOTOR_0x1FF_ID 0x1FF

// GM6020 id=5~7 (0x209~0x20B)
#define CAN_DJI_MOTOR_0x2FF_ID 0x2FF

#define ECD360 8192
#define ECD180 4096
#define ECD90 2048
#define ECD45 1024


//电机的数据
typedef struct {
    /* 实际电机反馈数据 */
    int16_t last_ecd;       //上一次的电机编码器计数值
    uint16_t ecd;           //转子机械角度, 电机编码器计数值
    int16_t speed_rpm;      //转子转速, 电机转速（每分钟转数，RPM）
    int16_t given_current;  //实际扭矩电流
    uint8_t temperate;      //电机温度

    /* 自定义数据 */
    int32_t total_ecd;      //电机旋转的总编码器数值
    uint16_t offset_ecd;    //电机的校准编码值
    int32_t round_cnt;      //电机旋转的总圈数
} DJI_Motor_t;

void DJI_info_update(DJI_Motor_t *motor, uint8_t *data);
void DJI_Round_Count(DJI_Motor_t *motor);
float DJI_Encoder_Limit(int16_t ecd);
float Motor_Ecd_To_Angle_Change(uint16_t ecd, uint16_t offset_ecd);


#endif


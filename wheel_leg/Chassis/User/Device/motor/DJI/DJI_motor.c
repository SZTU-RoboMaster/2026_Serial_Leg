#include "DJI_motor.h"


/**
 * @brief DJI电机解码
 * @param[in] motor  电机结构体指针
 * @param[in] data   接收到的数据的指针
 */
void DJI_info_update(DJI_Motor_t *motor, uint8_t *data) {
    motor->last_ecd = motor->ecd;
    /* 转子机械角度 */
    motor->ecd = (uint16_t)(data[0]<<8 | data[1]);
    /* 转子转速 */
    motor->speed_rpm = (int16_t)(data[2]<<8 | data[3]);
    /* 实际扭矩电流 */
    motor->given_current = (int16_t)(data[4]<<8 | data[5]);
    /* 电机温度 */
    motor->temperate = data[6];
}

/**
 * @brief 计算DJI电机转的圈数, 电机总编码值的计算, 解决过零点问题
 * @param[in] motor  电机结构体指针
 */
void DJI_Round_Count(DJI_Motor_t *motor) {
    if(motor->ecd - motor->last_ecd > ECD180){
        motor->round_cnt--;
    }
    else if(motor->ecd - motor->last_ecd < -ECD180)
    {
        motor->round_cnt++;
    }
    motor->total_ecd = motor->round_cnt*ECD360 + (motor->ecd - motor->offset_ecd);
}

/**
 * @brief DJI电机编码器限幅
 * @param[in] ecd  编码值
 * @return  编码值限幅后的值
 */
float DJI_Encoder_Limit(int16_t ecd) {
    while(ecd < 0 || ecd > ECD360) {
        if(ecd < 0) {
            ecd += ECD360;
        } else if(ecd > ECD360) {
            ecd -= ECD360;
        }
    }
    return (float)ecd;
}


/**
 * 计算距离零点的度数  -180-180
 * 根据电机编码器数据和偏移量计算相对角度变化的函数
 */
float Motor_Ecd_To_Angle_Change(uint16_t ecd, uint16_t offset_ecd) {
    int16_t tmp = 0;
    if(offset_ecd >= ECD180) {
        if(ecd > offset_ecd - ECD180) {
            tmp = ecd - offset_ecd;
        }
        else {
            tmp = ecd + ECD360 - offset_ecd;
        }
    }
    else {
        if(ecd > offset_ecd + ECD180) {
            tmp = ecd - ECD360 - offset_ecd;
        }
        else {
            tmp = ecd - offset_ecd;
        }
    }
    return (float)(tmp / 8192.f * 360);//TODO:必须要浮点数
}
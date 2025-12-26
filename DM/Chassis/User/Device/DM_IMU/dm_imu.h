#ifndef _DM_IMU_H
#define _DM_IMU_H

#include "stm32h7xx_hal.h"

#define RX_LEN   80

typedef struct {
    float accel[3];
    float gyro[3];
    float roll;
    float pitch;
    float yaw;
    float quaternion[4];

} dm_imu_t;


/** 加速度、角速度、姿态角的帧格式（每一帧都是19个字节） **/
#pragma (1)
typedef struct {
    // 帧头
    uint8_t header;

    uint8_t tag; // 固定为0xAA

    // 从机ID
    uint8_t slave_id;

    // 寄存器地址
    uint8_t reg; // 加速度：0x01  角速度：0x02  姿态角：0x03  四元数：0x04

    // 数据
    float data[3];

    // CRC16校验
    uint16_t crc;

    // 帧尾
    uint8_t tail;

} normal_packet_t;
#pragma ()

/** 四元数的帧格式（） **/
#pragma (1)
typedef struct {
    uint8_t header;
    uint8_t tag;
    uint8_t slave_id;
    uint8_t reg;
    float data[4];
    uint16_t crc;
    uint8_t tail;

} normal_ext_packet_t;
#pragma ()


void imu_data_unpack(uint8_t *pData);

extern dm_imu_t imu;

extern uint8_t uRx[RX_LEN];

#endif


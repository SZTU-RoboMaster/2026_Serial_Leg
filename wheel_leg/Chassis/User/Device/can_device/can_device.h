#ifndef CAN_DEVICE_H
#define CAN_DEVICE_H

#include <stdint-gcc.h>
#include <stdbool.h>

/** 发送报文ID **/
typedef enum{

    // 关节发送报文ID
    JOINT_LF_SEND = 0x01,
    JOINT_LB_SEND = 0x02,
    JOINT_RF_SEND = 0x03,
    JOINT_RB_SEND = 0x04,

    WHEEL_L_SEND = 0x05,
    WHEEL_R_SEND = 0x06,

} CanSendDeviceId;

/** 反馈报文ID **/
typedef enum{

    // 关节反馈报文ID
    JOINT_LF_RECEIVE = 0x11,
    JOINT_LB_RECEIVE = 0x12,
    JOINT_RF_RECEIVE = 0x13,
    JOINT_RB_RECEIVE = 0x14,

    WHEEL_L_RECEIVE = 0x205,
    WHEEL_R_RECEIVE = 0x206,

} CanReceiveDeviceId;

#endif

#ifndef BSP_CAN_H
#define BSP_CAN_H

#include <stdint-gcc.h>
#include "can_device.h"
#include "fdcan.h"

typedef struct {

    FDCAN_HandleTypeDef *hcan;
    FDCAN_TxHeaderTypeDef Header;
    uint8_t Data[8];

} FDCAN_TxFrame_TypeDef;

typedef struct {

    FDCAN_HandleTypeDef *hcan;
    FDCAN_RxHeaderTypeDef Header;
    uint8_t Data[8];

} FDCAN_RxFrame_TypeDef;

extern FDCAN_TxFrame_TypeDef LeftJointTxFrame;
extern FDCAN_TxFrame_TypeDef RightJointTxFrame;
extern FDCAN_TxFrame_TypeDef Board_Yaw_Wheel_TxFrame;

extern FDCAN_RxFrame_TypeDef FDCAN1_RxFrame;
extern FDCAN_RxFrame_TypeDef FDCAN2_RxFrame;
extern FDCAN_RxFrame_TypeDef FDCAN3_RxFrame;

void bsp_can_init(void);

#endif

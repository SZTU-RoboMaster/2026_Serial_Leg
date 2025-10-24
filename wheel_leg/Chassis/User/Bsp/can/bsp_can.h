#ifndef BSP_CAN_H
#define BSP_CAN_H

#include <stdint-gcc.h>
#include "can_device.h"
#include "can.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

typedef struct {

    CAN_HandleTypeDef 	*hcan;
    CAN_TxHeaderTypeDef  Header;
    uint8_t				 Data[8];

}CAN_TxFrame_TypeDef;

typedef struct {

    CAN_HandleTypeDef   *hcan;
    CAN_RxHeaderTypeDef  Header;
    uint8_t 			 Data[8];

} CAN_RxFrame_TypeDef;

extern CAN_TxFrame_TypeDef JointTxFrame;
extern CAN_TxFrame_TypeDef WheelTxFrame;

extern CAN_RxFrame_TypeDef CAN1_RxFrame;
extern CAN_RxFrame_TypeDef CAN2_RxFrame;

void can_filter_init(void);

void CAN_SendJointData(CAN_TxFrame_TypeDef* JointTxFrame);
void CAN_SendWheelData(CAN_TxFrame_TypeDef* WheelTxFrame);

#endif

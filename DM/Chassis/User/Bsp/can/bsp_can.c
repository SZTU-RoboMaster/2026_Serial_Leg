#include "bsp_can.h"

#include "wheel.h"
#include "joint.h"
#include "board_communication_task.h"
#include "DJI_motor.h"
#include "vofa.h"

/* 左侧关节发送结构体 */
FDCAN_TxFrame_TypeDef LeftJointTxFrame = {

        .hcan = &hfdcan1,

        .Header.Identifier = 0x00,
        .Header.IdType = FDCAN_STANDARD_ID,
        .Header.TxFrameType = FDCAN_DATA_FRAME,
        .Header.DataLength = 8,
        .Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE,
        .Header.BitRateSwitch = FDCAN_BRS_OFF,
        .Header.FDFormat =  FDCAN_CLASSIC_CAN,
        .Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS,
        .Header.MessageMarker = 0,
};

/* 右侧关节发送结构体 */
FDCAN_TxFrame_TypeDef RightJointTxFrame = {

        .hcan = &hfdcan2,

        .Header.Identifier = 0x00,
        .Header.IdType = FDCAN_STANDARD_ID,
        .Header.TxFrameType = FDCAN_DATA_FRAME,
        .Header.DataLength = 8,
        .Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE,
        .Header.BitRateSwitch = FDCAN_BRS_OFF,
        .Header.FDFormat =  FDCAN_CLASSIC_CAN,
        .Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS,
        .Header.MessageMarker = 0,
};

/* 板间通信 + 轮毂 + Yaw控制发送结构体 */
FDCAN_TxFrame_TypeDef Board_Yaw_Wheel_TxFrame = {

        .hcan = &hfdcan3,

        .Header.Identifier = 0x00,
        .Header.IdType = FDCAN_STANDARD_ID,
        .Header.TxFrameType = FDCAN_DATA_FRAME,
        .Header.DataLength = 8,
        .Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE,
        .Header.BitRateSwitch = FDCAN_BRS_OFF,
        .Header.FDFormat =  FDCAN_CLASSIC_CAN,
        .Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS,
        .Header.MessageMarker = 0,
};


/* 接收结构体 */
FDCAN_RxFrame_TypeDef FDCAN1_RxFrame;
FDCAN_RxFrame_TypeDef FDCAN2_RxFrame;
FDCAN_RxFrame_TypeDef FDCAN3_RxFrame;

void bsp_can_init(void) {

    /***************************************** FDCAN1 *****************************************************/
    FDCAN_FilterTypeDef FDCAN1_FilterConfig;

    FDCAN1_FilterConfig.IdType = FDCAN_STANDARD_ID;
    FDCAN1_FilterConfig.FilterIndex = 0;
    FDCAN1_FilterConfig.FilterType = FDCAN_FILTER_MASK;
    FDCAN1_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    FDCAN1_FilterConfig.FilterID1 = 0x00000000;
    FDCAN1_FilterConfig.FilterID2 = 0x00000000;

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN1_FilterConfig) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) !=
        HAL_OK) {
        Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        Error_Handler();
    }

/***************************************** FDCAN2 *****************************************************/
    FDCAN_FilterTypeDef FDCAN2_FilterConfig;

    FDCAN2_FilterConfig.IdType = FDCAN_STANDARD_ID;
    FDCAN2_FilterConfig.FilterIndex = 0;
    FDCAN2_FilterConfig.FilterType = FDCAN_FILTER_MASK;
    FDCAN2_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    FDCAN2_FilterConfig.FilterID1 = 0x00000000;
    FDCAN2_FilterConfig.FilterID2 = 0x00000000;

    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &FDCAN2_FilterConfig) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) !=
        HAL_OK) {
        Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK) {
        Error_Handler();
    }


    /***************************************** FDCAN3 *****************************************************/
    FDCAN_FilterTypeDef FDCAN3_FilterConfig;

    FDCAN3_FilterConfig.IdType = FDCAN_STANDARD_ID;
    FDCAN3_FilterConfig.FilterIndex = 0;
    FDCAN3_FilterConfig.FilterType = FDCAN_FILTER_MASK;
    FDCAN3_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    FDCAN3_FilterConfig.FilterID1 = 0x00000000;
    FDCAN3_FilterConfig.FilterID2 = 0x00000000;

    if (HAL_FDCAN_ConfigFilter(&hfdcan3, &FDCAN3_FilterConfig) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) !=
        HAL_OK) {
        Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_FDCAN_Start(&hfdcan3) != HAL_OK) {
        Error_Handler();
    }
}

/** FDCAN1接收中断处理 **/
static void FDCAN1_RxFifo0RxHandler(uint32_t *StdId, uint8_t Data[]) {

    switch (FDCAN1_RxFrame.Header.Identifier) {

        case JOINT_LF_RECEIVE: {
            dm8009p_info_update(&joint[LF], Data);
            break;
        }

        case JOINT_LB_RECEIVE: {
            dm8009p_info_update(&joint[LB], Data);
            break;
        }

        default: {
            break;
        }
    }

}

/** FDCAN2接收中断处理 **/
static void FDCAN2_RxFifo0RxHandler(uint32_t *StdId, uint8_t Data[]) {

    switch (FDCAN2_RxFrame.Header.Identifier) {
        case JOINT_RF_RECEIVE: {
            dm8009p_info_update(&joint[RF], Data);
            break;
        }

        case JOINT_RB_RECEIVE: {
            dm8009p_info_update(&joint[RB], Data);
            break;
        }

        default: {
            break;
        }
    }

}

/** FDCAN3接收中断处理 **/
static void FDCAN3_RxFifo0RxHandler(uint32_t *StdId, uint8_t Data[]) {

    switch (FDCAN3_RxFrame.Header.Identifier) {

        case WHEEL_L_RECEIVE: {
            DJI_Info_Update(&wheel[L], Data);
            break;
        }

        case WHEEL_R_RECEIVE: {
            DJI_Info_Update(&wheel[R], Data);
            break;
        }

        default: {
            break;
        }

    }
}


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {

    if (hfdcan == &hfdcan1) {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &FDCAN1_RxFrame.Header, FDCAN1_RxFrame.Data) == HAL_OK) {
            FDCAN1_RxFifo0RxHandler(&FDCAN1_RxFrame.Header.Identifier, FDCAN1_RxFrame.Data);
        }
    } else if (hfdcan == &hfdcan2) {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &FDCAN2_RxFrame.Header, FDCAN2_RxFrame.Data) == HAL_OK) {
            FDCAN2_RxFifo0RxHandler(&FDCAN2_RxFrame.Header.Identifier, FDCAN2_RxFrame.Data);
        }
    } else if (hfdcan == &hfdcan3) {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &FDCAN3_RxFrame.Header, FDCAN3_RxFrame.Data) == HAL_OK) {
            FDCAN3_RxFifo0RxHandler(&FDCAN3_RxFrame.Header.Identifier, FDCAN3_RxFrame.Data);
        }
    }
}


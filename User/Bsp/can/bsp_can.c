#include "bsp_can.h"
#include "wheel.h"
#include "joint.h"
#include "board_communication_task.h"
#include "DJI_motor.h"
#include "vofa.h"

/* =========================== 左关节 FDCAN发送结构体 =========================== */

FDCAN_TxFrame_TypeDef LeftJointTxFrame =
{
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

/* =========================== 右关节 FDCAN发送结构体 =========================== */

FDCAN_TxFrame_TypeDef RightJointTxFrame =
{
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

/* =========================== 轮毂 + Yaw + 板间通信 FDCAN发送结构体 =========================== */

FDCAN_TxFrame_TypeDef Board_Yaw_Wheel_TxFrame =
{
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

/* =========================== FDCAN接收结构体 =========================== */

FDCAN_RxFrame_TypeDef FDCAN1_RxFrame;
FDCAN_RxFrame_TypeDef FDCAN2_RxFrame;
FDCAN_RxFrame_TypeDef FDCAN3_RxFrame;

/* =========================== FDCA1初始化 =========================== */

static void FDCAN1_Config(void)
{
    FDCAN_FilterTypeDef FilterConfig;

    FilterConfig.IdType = FDCAN_STANDARD_ID;
    FilterConfig.FilterIndex = 0;
    FilterConfig.FilterType = FDCAN_FILTER_MASK;
    FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    FilterConfig.FilterID1 = 0x00000000;
    FilterConfig.FilterID2 = 0x00000000;

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &FilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) !=
        HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }
}

/* =========================== FDCAN2初始化 =========================== */

static void FDCAN2_Config(void)
{
    FDCAN_FilterTypeDef FilterConfig;

    FilterConfig.IdType = FDCAN_STANDARD_ID;
    FilterConfig.FilterIndex = 1; // ������������Ҫ�غ�
    FilterConfig.FilterType = FDCAN_FILTER_MASK;
    FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    FilterConfig.FilterID1 = 0x00000000;
    FilterConfig.FilterID2 = 0x00000000;

    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &FilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) !=
        HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* =========================== FDCAN3初始化 =========================== */

static void FDCAN3_Config(void)
{
    FDCAN_FilterTypeDef FilterConfig;

    FilterConfig.IdType = FDCAN_STANDARD_ID;
    FilterConfig.FilterIndex = 2;
    FilterConfig.FilterType = FDCAN_FILTER_MASK;
    FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    FilterConfig.FilterID1 = 0x00000000;
    FilterConfig.FilterID2 = 0x00000000;

    if (HAL_FDCAN_ConfigFilter(&hfdcan3, &FilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) !=
        HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_Start(&hfdcan3) != HAL_OK)
    {
        Error_Handler();
    }
}

/* =========================== FDCAN初始化 =========================== */

void bsp_can_init(void)
{

    /* =========================== FDCAN1初始化 =========================== */

    FDCAN1_Config();

    /* =========================== FDCAN2初始化 =========================== */

    FDCAN2_Config();

    /* =========================== FDCAN3初始化 =========================== */

    FDCAN3_Config();
}

/* =========================== FDCAN1接收中断 =========================== */

static void FDCAN1_RxFifo0RxHandler(uint32_t *StdId, uint8_t Data[])
{
    switch (FDCAN1_RxFrame.Header.Identifier)
    {
        /* =========================== 左前关节电机反馈解析 =========================== */

        case JOINT_LF_RECEIVE:
        {
            dm8009p_info_update(&joint[LF], Data);
            break;
        }

        /* =========================== 左后关节电机反馈解析 =========================== */

        case JOINT_LB_RECEIVE:
        {
            dm8009p_info_update(&joint[LB], Data);
            break;
        }
        default:
        {
            break;
        }
    }
}

/* =========================== FDCAN2接收中断 =========================== */

static void FDCAN2_RxFifo0RxHandler(uint32_t *StdId, uint8_t Data[])
{
    switch (FDCAN2_RxFrame.Header.Identifier)
    {

        /* =========================== 右前关节电机反馈解析 =========================== */

        case JOINT_RF_RECEIVE:
        {
            dm8009p_info_update(&joint[RF], Data);
            break;
        }

        /* =========================== 右后关节电机反馈解析 =========================== */

        case JOINT_RB_RECEIVE:
        {
            dm8009p_info_update(&joint[RB], Data);
            break;
        }
        default:
        {
            break;
        }
    }
}

/* =========================== FDCAN3接收中断 =========================== */

static void FDCAN3_RxFifo0RxHandler(uint32_t *StdId, uint8_t Data[])
{
    switch (FDCAN3_RxFrame.Header.Identifier)
    {

        /* =========================== 左轮毂DJI电机解码  =========================== */

        case WHEEL_L_RECEIVE:
        {
            DJI_Info_Update(&wheel[L], Data);
            break;
        }

        /* =========================== 右轮毂DJI电机解码 =========================== */

        case WHEEL_R_RECEIVE:
        {
            DJI_Info_Update(&wheel[R], Data);
            break;
        }

        /* =========================== yaw电机解码 =========================== */

        /* =========================== 板间通信解码 =========================== */

        default:
        {
            break;
        }
    }
}

/* =========================== CAN回调函数 =========================== */

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    /* =========================== FDCAN1回调函数 =========================== */

    if (hfdcan == &hfdcan1)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &FDCAN1_RxFrame.Header, FDCAN1_RxFrame.Data) == HAL_OK)
        {
            FDCAN1_RxFifo0RxHandler(&FDCAN1_RxFrame.Header.Identifier, FDCAN1_RxFrame.Data);
        }
    }

    /* =========================== FDCAN2回调函数 =========================== */

    if (hfdcan == &hfdcan2)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &FDCAN2_RxFrame.Header, FDCAN2_RxFrame.Data) == HAL_OK)
        {
            FDCAN2_RxFifo0RxHandler(&FDCAN2_RxFrame.Header.Identifier, FDCAN2_RxFrame.Data);
        }
    }

    /* =========================== FDCAN3回调函数 =========================== */

    if (hfdcan == &hfdcan3)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &FDCAN3_RxFrame.Header, FDCAN3_RxFrame.Data) == HAL_OK)
        {
            FDCAN3_RxFifo0RxHandler(&FDCAN3_RxFrame.Header.Identifier, FDCAN3_RxFrame.Data);
        }
    }
}


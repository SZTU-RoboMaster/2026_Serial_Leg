#include "bsp_usart.h"
#include "remote.h"
#include "dm_imu.h"
#include "vofa.h"

extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_uart5_rx;

// 非常重要！！！
__attribute__((section (".AXI_SRAM")))uint8_t SBUS_MultiRx_Buf[2][SBUS_RX_BUF_NUM];

void USART_RxDMA_MultiBuffer_Init(UART_HandleTypeDef *huart, uint32_t *DstAddress, uint32_t *SecondMemAddress,
                                  uint32_t DataLength) {

    huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;

    huart->RxXferSize = DataLength * 2;

    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

    do {
        __HAL_DMA_DISABLE(huart->hdmarx);
    } while (((DMA_Stream_TypeDef *) huart->hdmarx->Instance)->CR & DMA_SxCR_EN);

    /* Configure the source memory Buffer address  */
    ((DMA_Stream_TypeDef *) huart->hdmarx->Instance)->PAR = (uint32_t) &huart->Instance->RDR;

    /* Configure the destination memory Buffer address */
    ((DMA_Stream_TypeDef *) huart->hdmarx->Instance)->M0AR = (uint32_t) DstAddress;

    /* Configure DMA Stream destination address */
    ((DMA_Stream_TypeDef *) huart->hdmarx->Instance)->M1AR = (uint32_t) SecondMemAddress;

    /* Configure the length of data to be transferred from source to destination */
    ((DMA_Stream_TypeDef *) huart->hdmarx->Instance)->NDTR = DataLength;

    /* Enable double memory buffer */
    SET_BIT(((DMA_Stream_TypeDef *) huart->hdmarx->Instance)->CR, DMA_SxCR_DBM);

    /* Enable DMA */
    __HAL_DMA_ENABLE(huart->hdmarx);

}

static void USER_USART5_RxHandler(UART_HandleTypeDef *huart, uint16_t Size) {

    /* Current memory buffer used is Memory 0 */
    if (((((DMA_Stream_TypeDef *) huart->hdmarx->Instance)->CR) & DMA_SxCR_CT) == RESET) {

        /* Disable DMA */
        __HAL_DMA_DISABLE(huart->hdmarx);

        /* Switch Memory 0 to Memory 1*/
        ((DMA_Stream_TypeDef *) huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;

        /* Reset the receive count */
        __HAL_DMA_SET_COUNTER(huart->hdmarx, SBUS_RX_BUF_NUM * 2);

        /* Juge whether size is equal to the length of the received data */
        if (Size == SBUS_RX_BUF_NUM) {

            /* Memory 0 data update to remote_ctrl*/
            SBUS_TO_RC(SBUS_MultiRx_Buf[0], &remote_ctrl);

        }

    }
        /* Current memory buffer used is Memory 1 */
    else {
        /* Disable DMA */
        __HAL_DMA_DISABLE(huart->hdmarx);

        /* Switch Memory 1 to Memory 0*/
        ((DMA_Stream_TypeDef *) huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);

        /* Reset the receive count */
        __HAL_DMA_SET_COUNTER(huart->hdmarx, SBUS_RX_BUF_NUM * 2);

        if (Size == SBUS_RX_BUF_NUM) {
            /* Memory 1 to data update to remote_ctrl*/
            SBUS_TO_RC(SBUS_MultiRx_Buf[1], &remote_ctrl);
        }

    }

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart3) {
        imu_data_unpack(uRx);
    }

}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {

    if (huart == &huart5) {
        USER_USART5_RxHandler(huart, Size);
    }

    huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;

    /* Enalbe IDLE interrupt */
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

    /* Enable the DMA transfer for the receiver request */
    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

    /* Enable DMA */
    __HAL_DMA_ENABLE(huart->hdmarx);
}

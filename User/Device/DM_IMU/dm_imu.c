#include "dm_imu.h"
#include "usart.h"
#include <string.h>

Dm_imu_t DM_IMU;

__attribute__((section (".AXI_SRAM")))uint8_t uRx[RX_LEN];

void DM_IMU_Init(void)
{
    HAL_UART_Receive_DMA(&huart3, uRx, RX_LEN);
}

/* =========================== 达妙姿态解算 =========================== */

    void imu_data_unpack(uint8_t *pData)
    {
        normal_packet_t normal_packet;
        normal_ext_packet_t ext_packet;

        /********* 读取加速度 *********/
        memcpy(&normal_packet, pData + 0, 19);

        // 校验帧头、帧尾
        if ((normal_packet.header != 0x55) || (normal_packet.tail != 0x0A)) {
            return;
        }

        if (normal_packet.reg == 0x01) {

            DM_IMU.accel[X] = normal_packet.data[X];
            DM_IMU.accel[Y] = normal_packet.data[Y];
            DM_IMU.accel[Z] = normal_packet.data[Z];


        }

        /********* 读取角速度 *********/
        memcpy(&normal_packet, pData + 19, 19);

        if ((normal_packet.header != 0x55) || (normal_packet.tail != 0x0A)) {
            return;
        }

        /** 角速度 **/
        if (normal_packet.reg == 0x02) {

            DM_IMU.gyro[X] = normal_packet.data[X];
            DM_IMU.gyro[Y] = normal_packet.data[Y];
            DM_IMU.gyro[Z] = normal_packet.data[Z];
        }

        /********* 读取姿态角 *********/
        memcpy(&normal_packet, pData + 38, 19); // 19 * 2

        if ((normal_packet.header != 0x55) || (normal_packet.tail != 0x0A)) {
            return;
        }

        if (normal_packet.reg == 0x03) {

            DM_IMU.roll = normal_packet.data[X];
            DM_IMU.pitch = normal_packet.data[Y];
            DM_IMU.yaw = normal_packet.data[Z];
        }

        /********* 读取四元数 *********/
        memcpy(&ext_packet, pData + 57, 23); // 19 * 3

        if ((ext_packet.header != 0x55) || (ext_packet.tail != 0x0A)) {
            return;
        }

        /** 四元数 **/
        if (ext_packet.reg == 0x04) {

            DM_IMU.quaternion[0] = ext_packet.data[0];
            DM_IMU.quaternion[1] = ext_packet.data[1];
            DM_IMU.quaternion[2] = ext_packet.data[2];
            DM_IMU.quaternion[3] = ext_packet.data[3];
        }

    }

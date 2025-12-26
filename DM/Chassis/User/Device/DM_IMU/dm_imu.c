#include "dm_imu.h"
#include <string.h>

dm_imu_t imu;

__attribute__((section (".AXI_SRAM")))uint8_t uRx[RX_LEN];

void imu_data_unpack(uint8_t *pData) {
    normal_packet_t normal_packet;
    normal_ext_packet_t ext_packet;

    /********* 读取加速度 *********/
    memcpy(&normal_packet, pData + 0, 19);

    // 校验帧头、帧尾
    if ((normal_packet.header != 0x55) || (normal_packet.tail != 0x0A)) {
        return;
    }

    if (normal_packet.reg == 0x01) {
        imu.accel[0] = normal_packet.data[0];
        imu.accel[1] = normal_packet.data[1];
        imu.accel[2] = normal_packet.data[2];
    }

    /********* 读取角速度 *********/
    memcpy(&normal_packet, pData + 19, 19);

    if ((normal_packet.header != 0x55) || (normal_packet.tail != 0x0A)) {
        return;
    }

    /** 角速度 **/
    if (normal_packet.reg == 0x02) {
        imu.gyro[0] = normal_packet.data[0];
        imu.gyro[1] = normal_packet.data[1];
        imu.gyro[2] = normal_packet.data[2];
    }

    /********* 读取姿态角 *********/
    memcpy(&normal_packet, pData + 38, 19); // 19 * 2

    if ((normal_packet.header != 0x55) || (normal_packet.tail != 0x0A)) {
        return;
    }

    if (normal_packet.reg == 0x03) {
        imu.roll = normal_packet.data[0];
        imu.pitch = normal_packet.data[1];
        imu.yaw = normal_packet.data[2];
    }

    /********* 读取四元数 *********/
    memcpy(&ext_packet, pData + 57, 23); // 19 * 3

    if ((ext_packet.header != 0x55) || (ext_packet.tail != 0x0A)) {
        return;
    }

    /** 四元数 **/
    if (ext_packet.reg == 0x04) {
        imu.quaternion[0] = ext_packet.data[0];
        imu.quaternion[1] = ext_packet.data[1];
        imu.quaternion[2] = ext_packet.data[2];
        imu.quaternion[3] = ext_packet.data[3];
    }

}
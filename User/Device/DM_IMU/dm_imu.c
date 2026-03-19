#include "dm_imu.h"
#include "usart.h"
#include <string.h>

#include "QuaternionEKF.h"
#include "robot_def.h"

Dm_imu_t DM_IMU;
Dm_imu_t chassis_imu_reference;

__attribute__((section (".AXI_SRAM")))uint8_t uRx[RX_LEN];

void DM_IMU_Init(void)
{
    HAL_UART_Receive_DMA(&huart3, uRx, RX_LEN);
}

/* =========================== IMU 位置补偿 (杠杆臂效应补偿) ============================= */
/* 补偿公式：a_compensated = a_raw - [ω_dot × r + ω × (ω × r)] */
/* 其中 r 是 IMU 相对于质心的位置向量 */

void IMU_Position_Compensation(float *accel_raw, float *gyro, float *gyro_dot)
{
    float r[3] = {
        chassis_physical_config.imu_pos_x,
        chassis_physical_config.imu_pos_y,
        chassis_physical_config.imu_pos_z
    };

    // 1. 计算切向加速度：ω_dot × r
    float tangential_accel[3];
    tangential_accel[0] = gyro_dot[1] * r[2] - gyro_dot[2] * r[1];
    tangential_accel[1] = gyro_dot[2] * r[0] - gyro_dot[0] * r[2];
    tangential_accel[2] = gyro_dot[0] * r[1] - gyro_dot[1] * r[0];

    // 2. 计算向心加速度：ω × (ω × r)
    // 先计算 ω × r
    float omega_cross_r[3];
    omega_cross_r[0] = gyro[1] * r[2] - gyro[2] * r[1];
    omega_cross_r[1] = gyro[2] * r[0] - gyro[0] * r[2];
    omega_cross_r[2] = gyro[0] * r[1] - gyro[1] * r[0];

    // 再计算 ω × (ω × r)
    float centripetal_accel[3];
    centripetal_accel[0] = gyro[1] * omega_cross_r[2] - gyro[2] * omega_cross_r[1];
    centripetal_accel[1] = gyro[2] * omega_cross_r[0] - gyro[0] * omega_cross_r[2];
    centripetal_accel[2] = gyro[0] * omega_cross_r[1] - gyro[1] * omega_cross_r[0];

    // 3. 补偿：a_compensated = a_raw - (tangential + centripetal)
    accel_raw[0] -= (tangential_accel[0] + centripetal_accel[0]);
    accel_raw[1] -= (tangential_accel[1] + centripetal_accel[1]);
    accel_raw[2] -= (tangential_accel[2] + centripetal_accel[2]);
}

/* =========================== 达妙姿态解算 =========================== */

    void imu_data_unpack(uint8_t *pData)
    {
    static float last_gyro[3] = {0, 0, 0};
    /* iss：根据实际 IMU 更新频率调整 */
    float dt = 0.001f;
    float gyro_dot[3];
    static bool first_run = true;


        normal_packet_t normal_packet;
        normal_ext_packet_t ext_packet;

        /********* 读取加速度 *********/
        memcpy(&normal_packet, pData + 0, 19);

        // 校验帧头、帧尾
        if ((normal_packet.header != 0x55) || (normal_packet.tail != 0x0A)) {
            return;
        }

        if (normal_packet.reg == 0x01) {

            DM_IMU.accel[0] = normal_packet.data[0];
            DM_IMU.accel[1] = normal_packet.data[1];
            DM_IMU.accel[2] = normal_packet.data[2];

            // 新增：更新EKF加速度数据
            QEKF_INS.Accel[0] = DM_IMU.accel[0];
            QEKF_INS.Accel[1] = DM_IMU.accel[1];
            QEKF_INS.Accel[2] = DM_IMU.accel[2];

        }

        /********* 读取角速度 *********/
        memcpy(&normal_packet, pData + 19, 19);

        if ((normal_packet.header != 0x55) || (normal_packet.tail != 0x0A)) {
            return;
        }

        /** 角速度 **/
        if (normal_packet.reg == 0x02) {

            DM_IMU.gyro[0] = normal_packet.data[0];
            DM_IMU.gyro[1] = normal_packet.data[1];
            DM_IMU.gyro[2] = normal_packet.data[2];
        }

         // 计算角加速度
            if (first_run) {
                gyro_dot[0] = gyro_dot[1] = gyro_dot[2] = 0.0f;
                first_run = false;
            } else {
                gyro_dot[0] = (DM_IMU.gyro[0] - last_gyro[0]) / dt;
                gyro_dot[1] = (DM_IMU.gyro[1] - last_gyro[1]) / dt;
                gyro_dot[2] = (DM_IMU.gyro[2] - last_gyro[2]) / dt;
            }

            // 位置补偿
            IMU_Position_Compensation(DM_IMU.accel, DM_IMU.gyro, gyro_dot);

            // 更新上次值
            last_gyro[0] = DM_IMU.gyro[0];
            last_gyro[1] = DM_IMU.gyro[1];
            last_gyro[2] = DM_IMU.gyro[2];

        /********* 读取姿态角 *********/
        memcpy(&normal_packet, pData + 38, 19); // 19 * 2

        if ((normal_packet.header != 0x55) || (normal_packet.tail != 0x0A)) {
            return;
        }

        if (normal_packet.reg == 0x03) {

            DM_IMU.roll = normal_packet.data[0];
            DM_IMU.pitch = normal_packet.data[1];
            DM_IMU.yaw = normal_packet.data[2];
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

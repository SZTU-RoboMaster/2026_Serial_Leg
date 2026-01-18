#include <stdint-gcc.h>
#include "ins_task.h"
#include "dm_imu.h"
#include "bsp_dwt.h"
#include "vofa.h"
#include "robot_def.h"

INS_t INS;

// 机体坐标系下xyz轴的单位向量
const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};

// 重力加速度在世界坐标系下的表示
const float gravity[3] = {0, 0, 9.81f};

uint32_t INS_DWT_Count = 0;
static float dt = 0, t = 0;


/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q) {
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}


/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
static void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q) {
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

void INS_Init(void) {
    INS.AccelLPF = 0.0085f;
}

void Body_Accel_To_Earth(void) {

    dt = DWT_GetDeltaT(&INS_DWT_Count);
    t += dt;

    // 更新加速度
    INS.Accel[X] = DM_IMU.accel[X];
    INS.Accel[Y] = DM_IMU.accel[Y];
    INS.Accel[Z] = DM_IMU.accel[Z];

    // 更新四元数
    INS.q[0] = DM_IMU.quaternion[0];
    INS.q[1] = DM_IMU.quaternion[1];
    INS.q[2] = DM_IMU.quaternion[2];
    INS.q[3] = DM_IMU.quaternion[3];


    // 机体系基向量转换到导航坐标系，选取惯性系为导航系
    BodyFrameToEarthFrame(xb, INS.xn, INS.q);
    BodyFrameToEarthFrame(yb, INS.yn, INS.q);
    BodyFrameToEarthFrame(zb, INS.zn, INS.q);

    // 将重力加速度从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
    float gravity_b[3];
    EarthFrameToBodyFrame(gravity, gravity_b, INS.q);

    for (uint8_t i = 0; i < 3; i++) // 过一个低通滤波
    {
        INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * dt / (INS.AccelLPF + dt) +
                               INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
    }

    BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q); // 转换回导航系n

    chassis.imu_reference.robot_ax = -INS.MotionAccel_n[Y];
}
//
// Created by 19108 on 2026/3/7.
//

#include "phi0_kalman_filter.h"
#include "kalman_filter.h"
#include "wheel.h"
#include "robot_def.h"
#include "vofa.h"

/* =========================== 卡尔曼滤波器结构体 ============================= */

    KalmanFilter_t PHI0_EstimateKF;

/* =========================== 初始化P ============================= */

    float PHI0_EstimateKF_P[4] = { 1.0f , 0.0f ,
                                    0.0f , 1.0f };

/* =========================== 设置 F Q R ============================= */

    /* iss：要调 */

    float PHI0_EstimateKF_F[4] = { 1.0f , CHASSIS_PERIOD * 0.001f ,
                                    0.0f ,                    1.0f };
    float PHI0_EstimateKF_Q[4] = { PHI0_PROCESS_NOISE ,              0.0f ,
                                                 0.0f , PHI0_dot_PROCESS_NOISE };
    float PHI0_EstimateKF_R[4] = { PHI0_MEASURE_NOISE ,              0.0f ,
                                                 0.0f , PHI0_dot_MEASURE_NOISE };
    const float PHI0_EstimateKF_H[4] = { 1.0f , 0.0f ,
                                          0.0f , 1.0f };

/* =========================== phi0 phi0'融合观测器初始化 ============================= */

    void PHI0_EstimateKF_Init(KalmanFilter_t *PHI0_EstimateKF)
    {

        /* =========================== 初始化卡尔曼结构体 状态向量2维 没有控制量 测量向量2维 ============================= */

            Kalman_Filter_Init(PHI0_EstimateKF, 2, 0, 2);

        /* =========================== 把该开头定义的矩阵复制到结构体中的矩阵 ============================= */

            /* =========================== 结构体矩阵是动态的 不能直接赋值 ============================= */

            memcpy(PHI0_EstimateKF->F_data, PHI0_EstimateKF_F, sizeof(PHI0_EstimateKF_F));
            memcpy(PHI0_EstimateKF->P_data, PHI0_EstimateKF_P, sizeof(PHI0_EstimateKF_P));
            memcpy(PHI0_EstimateKF->Q_data, PHI0_EstimateKF_Q, sizeof(PHI0_EstimateKF_Q));
            memcpy(PHI0_EstimateKF->R_data, PHI0_EstimateKF_R, sizeof(PHI0_EstimateKF_R));
            memcpy(PHI0_EstimateKF->H_data, PHI0_EstimateKF_H, sizeof(PHI0_EstimateKF_H));

    }

/* =========================== 2.4 phi0 phi0'融合 =========================== */

    void PHI0_KF_calc(KalmanFilter_t *PHI0_EstimateKF, Leg *leg, float phi0, float phi0_dot)
    {

        /* =========================== 卡尔曼滤波器测量值更新 =========================== */

            PHI0_EstimateKF->MeasuredVector[0] = phi0;
            PHI0_EstimateKF->MeasuredVector[1] = phi0_dot;

        /* =========================== 卡尔曼滤波器更新 =========================== */

            Kalman_Filter_Update(PHI0_EstimateKF);

        /* =========================== 设置为机体x' =========================== */

            leg->vmc.forward_kinematics.fk_phi.phi0 = PHI0_EstimateKF->FilteredValue[0];
            leg->vmc.forward_kinematics.fk_phi.d_phi0 = PHI0_EstimateKF->FilteredValue[1];
    }
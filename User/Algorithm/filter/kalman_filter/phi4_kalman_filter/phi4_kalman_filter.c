//
// Created by 19108 on 2026/3/7.
//

#include "phi4_kalman_filter.h"
#include "kalman_filter.h"
#include "wheel.h"
#include "robot_def.h"
#include "vofa.h"

/* =========================== 卡尔曼滤波器结构体 ============================= */

    KalmanFilter_t PHI4_EstimateKF;

/* =========================== 初始化P ============================= */

    float PHI4_EstimateKF_P[4] = { 1.0f , 0.0f ,
                                    0.0f , 1.0f };

/* =========================== 设置 F Q R ============================= */

    /* iss：要调 */

    float PHI4_EstimateKF_F[4] = { 1.0f , CHASSIS_PERIOD * 0.001f ,
                                    0.0f ,                    1.0f };
    float PHI4_EstimateKF_Q[4] = { PHI4_PROCESS_NOISE ,              0.0f ,
                                                 0.0f , PHI4_dot_PROCESS_NOISE };
    float PHI4_EstimateKF_R[4] = { PHI4_MEASURE_NOISE ,              0.0f ,
                                                 0.0f , PHI4_dot_MEASURE_NOISE };
    const float PHI4_EstimateKF_H[4] = { 1.0f , 0.0f ,
                                          0.0f , 1.0f };

/* =========================== phi4 phi4'融合观测器初始化 ============================= */

    void PHI4_EstimateKF_Init(KalmanFilter_t *PHI4_EstimateKF)
    {

        /* =========================== 初始化卡尔曼结构体 状态向量2维 没有控制量 测量向量2维 ============================= */

            Kalman_Filter_Init(PHI4_EstimateKF, 2, 0, 2);

        /* =========================== 把该开头定义的矩阵复制到结构体中的矩阵 ============================= */

            /* =========================== 结构体矩阵是动态的 不能直接赋值 ============================= */

            memcpy(PHI4_EstimateKF->F_data, PHI4_EstimateKF_F, sizeof(PHI4_EstimateKF_F));
            memcpy(PHI4_EstimateKF->P_data, PHI4_EstimateKF_P, sizeof(PHI4_EstimateKF_P));
            memcpy(PHI4_EstimateKF->Q_data, PHI4_EstimateKF_Q, sizeof(PHI4_EstimateKF_Q));
            memcpy(PHI4_EstimateKF->R_data, PHI4_EstimateKF_R, sizeof(PHI4_EstimateKF_R));
            memcpy(PHI4_EstimateKF->H_data, PHI4_EstimateKF_H, sizeof(PHI4_EstimateKF_H));

    }

/* =========================== 2.4 phi4 phi4'融合 =========================== */

    void PHI4_KF_calc(KalmanFilter_t *PHI4_EstimateKF, Leg *leg, float phi4, float phi4_dot)
    {

        /* =========================== 卡尔曼滤波器测量值更新 =========================== */

            PHI4_EstimateKF->MeasuredVector[0] = phi4;
            PHI4_EstimateKF->MeasuredVector[1] = phi4_dot;

        /* =========================== 卡尔曼滤波器更新 =========================== */

            Kalman_Filter_Update(PHI4_EstimateKF);

        /* =========================== 设置为机体phi4 phi4' =========================== */

            leg->vmc.forward_kinematics.fk_phi.phi4 = PHI4_EstimateKF->FilteredValue[0];
            leg->vmc.forward_kinematics.fk_phi.phi4_dot = PHI4_EstimateKF->FilteredValue[1];
    }
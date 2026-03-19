//
// Created by 19108 on 2026/3/7.
//

#include "L0_kalman_filter.h"
#include "wheel.h"
#include "robot_def.h"
#include "vofa.h"

/* =========================== 卡尔曼滤波器结构体 ============================= */

    KalmanFilter_t L0_EstimateKF;

/* =========================== 初始化P ============================= */

    float L0_EstimateKF_P[4] = { 1.0f , 0.0f ,
                                    0.0f , 1.0f };

/* =========================== 设置 F Q R ============================= */

    /* iss：要调 */

    float L0_EstimateKF_F[4] = { 1.0f , CHASSIS_PERIOD * 0.001f ,
                                    0.0f ,                    1.0f };
    float L0_EstimateKF_Q[4] = { L0_PROCESS_NOISE ,              0.0f ,
                                                 0.0f , LO_dot_PROCESS_NOISE };
    float L0_EstimateKF_R[4] = { L0_MEASURE_NOISE ,              0.0f ,
                                                 0.0f , L0_dot_MEASURE_NOISE };
    const float L0_EstimateKF_H[4] = { 1.0f , 0.0f ,
                                          0.0f , 1.0f };

/* =========================== L0 L0' 融合观测器初始化 ============================= */

    void L0_EstimateKF_Init(KalmanFilter_t *L0_EstimateKF)
    {

        /* =========================== 初始化卡尔曼结构体 状态向量2维 没有控制量 测量向量2维 ============================= */

            Kalman_Filter_Init(L0_EstimateKF, 2, 0, 2);

        /* =========================== 把该开头定义的矩阵复制到结构体中的矩阵 ============================= */

            /* =========================== 结构体矩阵是动态的 不能直接赋值 ============================= */

            memcpy(L0_EstimateKF->F_data, L0_EstimateKF_F, sizeof(L0_EstimateKF_F));
            memcpy(L0_EstimateKF->P_data, L0_EstimateKF_P, sizeof(L0_EstimateKF_P));
            memcpy(L0_EstimateKF->Q_data, L0_EstimateKF_Q, sizeof(L0_EstimateKF_Q));
            memcpy(L0_EstimateKF->R_data, L0_EstimateKF_R, sizeof(L0_EstimateKF_R));
            memcpy(L0_EstimateKF->H_data, L0_EstimateKF_H, sizeof(L0_EstimateKF_H));

    }

/* =========================== 2.4 L0 LO'融合 =========================== */

    void L0_KF_calc(KalmanFilter_t *L0_EstimateKF, Leg *leg, float L0, float L0_dot)
    {

        /* =========================== 卡尔曼滤波器测量值更新 =========================== */

            L0_EstimateKF->MeasuredVector[0] = L0;
            L0_EstimateKF->MeasuredVector[1] = L0_dot;

        /* =========================== 卡尔曼滤波器更新 =========================== */

            Kalman_Filter_Update(L0_EstimateKF);

        /* =========================== 设置为机体L0 L0' =========================== */

            leg->vmc.forward_kinematics.fk_L0.L0 = L0_EstimateKF->FilteredValue[0];
            leg->vmc.forward_kinematics.fk_L0.L0_dot = L0_EstimateKF->FilteredValue[1];
    }
#include "speed_kalman_filter.h"

#include <stdio.h>

#include "wheel.h"
#include "robot_def.h"
#include "vofa.h"

/* =========================== 卡尔曼滤波器结构体 ============================= */

    KalmanFilter_t Speed_EstimateKF;

/* =========================== 轮毂速度与加速度融合后的结果 ============================= */

    float vel_acc[2];

/* =========================== 初始化P ============================= */

    float Speed_EstimateKF_P[4] = { 1.0f , 0.0f ,
                                    0.0f , 1.0f };

/* =========================== 设置 F Q R ============================= */

    /* iss：要调 */

    float Speed_EstimateKF_F[4] = { 1.0f , CHASSIS_PERIOD * 0.001f ,
                                    0.0f ,                    1.0f };
    float Speed_EstimateKF_Q[4] = { VEL_PROCESS_NOISE ,              0.0f ,
                                                 0.0f , ACC_PROCESS_NOISE };
    float Speed_EstimateKF_R[4] = { VEL_MEASURE_NOISE ,              0.0f ,
                                                 0.0f , ACC_MEASURE_NOISE };
    const float Speed_EstimateKF_H[4] = { 1.0f , 0.0f ,
                                          0.0f , 1.0f };

/* =========================== x' x''融合观测器初始化 ============================= */

    void Speed_EstimateKF_Init(KalmanFilter_t *Speed_EstimateKF)
    {

        /* =========================== 初始化卡尔曼结构体 状态向量2维 没有控制量 测量向量2维 ============================= */

            Kalman_Filter_Init(Speed_EstimateKF, 2, 0, 2);

        /* =========================== 把该开头定义的矩阵复制到结构体中的矩阵 ============================= */

            /* =========================== 结构体矩阵是动态的 不能直接赋值 ============================= */

            memcpy(Speed_EstimateKF->F_data, Speed_EstimateKF_F, sizeof(Speed_EstimateKF_F));
            memcpy(Speed_EstimateKF->P_data, Speed_EstimateKF_P, sizeof(Speed_EstimateKF_P));
            memcpy(Speed_EstimateKF->Q_data, Speed_EstimateKF_Q, sizeof(Speed_EstimateKF_Q));
            memcpy(Speed_EstimateKF->R_data, Speed_EstimateKF_R, sizeof(Speed_EstimateKF_R));
            memcpy(Speed_EstimateKF->H_data, Speed_EstimateKF_H, sizeof(Speed_EstimateKF_H));

    }

/* =========================== 2.4 x' x''融合 估计x' =========================== */

    void Speed_KF_calc(KalmanFilter_t *Speed_EstimateKF, Leg *leg, float vel, float acc)
    {

        /* =========================== 卡尔曼滤波器测量值更新 =========================== */

            Speed_EstimateKF->MeasuredVector[0] = vel;
            Speed_EstimateKF->MeasuredVector[1] = acc;

        /* =========================== 卡尔曼滤波器更新 =========================== */

            Kalman_Filter_Update(Speed_EstimateKF);

        /* =========================== 设置为机体x' =========================== */

            leg->state_variable_feedback.x_dot = Speed_EstimateKF->FilteredValue[0];

    }

#include "theta_kalman_filter.h"
#include "vofa.h"

/* =========================== 卡尔曼滤波器结构体 ============================= */

    KalmanFilter_t Theta_EstimateKF;

/* =========================== 初始化P ============================= */

    float Theta_EstimateKF_P[4] = { 1.0f , 0.0f ,
                                0.0f , 1.0f };

/* =========================== 设置 F Q R ============================= */

    /* iss：要调 */

    float Theta_EstimateKF_F[4] = { 1.0f , CHASSIS_PERIOD * 0.001f ,
                                    0.0f ,                    1.0f };
    float Theta_EstimateKF_Q[4] = { THETA_PROCESS_NOISE ,              0.0f ,
                                                 0.0f , THETA_DOT_PROCESS_NOISE };
    float Theta_EstimateKF_R[4] = { THETA_MEASURE_NOISE ,              0.0f ,
                                                 0.0f , THETA_DOT_MEASURE_NOISE };
    const float Theta_EstimateKF_H[4] = { 1.0f , 0.0f ,
                                          0.0f , 1.0f };

/* =========================== θ θ'融合观测器初始化 ============================= */

    void Theta_EstimateKF_Init(KalmanFilter_t *Theta_EstimateKF)
    {

        /* =========================== 初始化卡尔曼结构体 状态向量2维 没有控制量 测量向量2维 =========================== */

            Kalman_Filter_Init(Theta_EstimateKF, 2, 0, 2);

        /* =========================== 把上面定义的矩阵复制到结构体中的矩阵 =========================== */

        /* =========================== 结构体矩阵是动态的 不能直接赋值 =========================== */

            memcpy(Theta_EstimateKF->F_data, Theta_EstimateKF_F, sizeof(Theta_EstimateKF_F));
            memcpy(Theta_EstimateKF->P_data, Theta_EstimateKF_P, sizeof(Theta_EstimateKF_P));
            memcpy(Theta_EstimateKF->Q_data, Theta_EstimateKF_Q, sizeof(Theta_EstimateKF_Q));
            memcpy(Theta_EstimateKF->R_data, Theta_EstimateKF_R, sizeof(Theta_EstimateKF_R));
            memcpy(Theta_EstimateKF->H_data, Theta_EstimateKF_H, sizeof(Theta_EstimateKF_H));

    }

/* =========================== 2.4 θ θ' 融合 估计 θ θ' =========================== */

    void theta_KF_calc(KalmanFilter_t *Theta_EstimateKF, Leg *leg, float theta, float theta_dot)
    {

        /* =========================== 卡尔曼滤波器测量值更新 =========================== */

            Theta_EstimateKF->MeasuredVector[0] = theta;
            Theta_EstimateKF->MeasuredVector[1] = theta_dot;

        /* =========================== 卡尔曼滤波器更新 =========================== */

            Kalman_Filter_Update(Theta_EstimateKF);

        /* ===========================  设置为机体θ θ' =========================== */

            leg->state_variable_feedback.theta = Theta_EstimateKF->FilteredValue[0];
            leg->state_variable_feedback.theta_dot = Theta_EstimateKF->FilteredValue[1];

    }
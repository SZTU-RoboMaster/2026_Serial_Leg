#include "theta_kalman_filter.h"
#include "vofa.h"

/** Theta摆角-角速度融合 **/
KalmanFilter_t Theta_EstimateKF;       // 卡尔曼滤波器结构体

float Theta_EstimateKF_F[4] = {1.0f, CHASSIS_PERIOD * 0.001f,
                               0.0f, 1.0f};       // 状态转移矩阵，控制周期为0.001s

float Theta_EstimateKF_P[4] = {1.0f, 0.0f,
                               0.0f, 1.0f};    // 后验估计协方差初始值

float Theta_EstimateKF_Q[4] = {THETA_PROCESS_NOISE, 0.0f,
                               0.0f, THETA_DOT_PROCESS_NOISE};    // Q矩阵初始值、先验估计值方差噪声

float Theta_EstimateKF_R[4] = {THETA_MEASURE_NOISE, 0.0f,
                               0.0f, THETA_DOT_MEASURE_NOISE};    // 测量噪声方差

const float Theta_EstimateKF_H[4] = {1.0f, 0.0f,
                                     0.0f, 1.0f};    // 设置矩阵H为常量


/*******************************************************************************
 *                                  摆角-角速度融合                              *
 *******************************************************************************/
void Theta_EstimateKF_Init(KalmanFilter_t *Theta_EstimateKF)//初始化卡尔曼结构体，并把该开头定义的矩阵复制到结构体中的矩阵
{
    Kalman_Filter_Init(Theta_EstimateKF, 2, 0, 2);    // 状态向量2维 没有控制量 测量向量2维

    memcpy(Theta_EstimateKF->F_data, Theta_EstimateKF_F, sizeof(Theta_EstimateKF_F));
    memcpy(Theta_EstimateKF->P_data, Theta_EstimateKF_P, sizeof(Theta_EstimateKF_P));
    memcpy(Theta_EstimateKF->Q_data, Theta_EstimateKF_Q, sizeof(Theta_EstimateKF_Q));
    memcpy(Theta_EstimateKF->R_data, Theta_EstimateKF_R, sizeof(Theta_EstimateKF_R));
    memcpy(Theta_EstimateKF->H_data, Theta_EstimateKF_H, sizeof(Theta_EstimateKF_H));

}

static void Theta_EstimateKF_Update(KalmanFilter_t *Theta_EstimateKF, float theta, float theta_dot) {
    //卡尔曼滤波器测量值更新
    Theta_EstimateKF->MeasuredVector[0] = theta;//测量角度
    Theta_EstimateKF->MeasuredVector[1] = theta_dot;//测量角速度

    //卡尔曼滤波器更新函数
    Kalman_Filter_Update(Theta_EstimateKF);

}

void theta_KF_calc(Leg *leg, float theta, float theta_dot) {

    Theta_EstimateKF_Update(&Theta_EstimateKF, theta, theta_dot);

    leg->state_variable_feedback.theta = Theta_EstimateKF.FilteredValue[0];
    leg->state_variable_feedback.theta_dot = Theta_EstimateKF.FilteredValue[1];

}
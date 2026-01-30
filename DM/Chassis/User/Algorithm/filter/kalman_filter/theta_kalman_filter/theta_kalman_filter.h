#ifndef _THETA_KALMAN_FILTER_H
#define _THETA_KALMAN_FILTER_H

#include "kalman_filter.h"
#include "robot_def.h"

// Q
#define THETA_PROCESS_NOISE 1000.0f      // 摆角过程噪声
#define THETA_DOT_PROCESS_NOISE 1000.0f  // 摆角角速度过程噪声

// R
#define THETA_MEASURE_NOISE 10.0f      // 摆角测量噪声  起码要确保角度不失真
#define THETA_DOT_MEASURE_NOISE 1000.0f  // 摆角角速度测量噪声

void Theta_EstimateKF_Init(KalmanFilter_t *Theta_EstimateKF);

void theta_KF_calc(Leg *leg, float theta, float theta_dot);

extern KalmanFilter_t Theta_EstimateKF;

#endif
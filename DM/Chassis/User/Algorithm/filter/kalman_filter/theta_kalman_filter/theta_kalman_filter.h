#ifndef _THETA_KALMAN_FILTER_H
#define _THETA_KALMAN_FILTER_H

#include "kalman_filter.h"
#include "robot_def.h"

#define THETA_PROCESS_NOISE 1.0f      // 摆角过程噪声
#define THETA_DOT_PROCESS_NOISE 1.0f  // 摆角角速度过程噪声

#define THETA_MEASURE_NOISE 0.1f      // 摆角测量噪声
#define THETA_DOT_MEASURE_NOISE 0.1f  // 摆角角速度测量噪声

void Theta_EstimateKF_Init(KalmanFilter_t *Theta_EstimateKF);

float theta_calc(Leg *leg, float theta, float theta_dot);

extern KalmanFilter_t Theta_EstimateKF;

#endif
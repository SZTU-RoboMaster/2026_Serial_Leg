#ifndef _KALMAN_FILTER_H
#define _KALMAN_FILTER_H

#include "kalman_filter.h"

#define VEL_PROCESS_NOISE 20.0f   // 速度过程噪声 20
#define ACC_PROCESS_NOISE 100.0f  // 加速度过程噪声 100

#define VEL_MEASURE_NOISE 100.0f  // 速度测量噪声 100
#define ACC_MEASURE_NOISE 0.01f  // 加速度测量噪声 0.01

void xvEstimateKF_Init(KalmanFilter_t *EstimateKF);

void speed_calc(void);

extern KalmanFilter_t vaEstimateKF;

extern float vel_acc[2];

#endif
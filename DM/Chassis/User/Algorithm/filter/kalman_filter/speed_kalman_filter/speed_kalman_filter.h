#ifndef _SPEED_KALMAN_FILTER_H
#define _SPEED_KALMAN_FILTER_H

#include "kalman_filter.h"

#define VEL_PROCESS_NOISE 1.0f   // 速度过程噪声 20
#define ACC_PROCESS_NOISE 1.0f  // 加速度过程噪声 100

#define VEL_MEASURE_NOISE 1.0f  // 速度测量噪声 100
#define ACC_MEASURE_NOISE 1.0f  // 加速度测量噪声 0.01

void Speed_EstimateKF_Init(KalmanFilter_t *EstimateKF);

void speed_calc(void);

extern KalmanFilter_t Speed_EstimateKF;

extern float vel_acc[2];

#endif
#ifndef _SPEED_KALMAN_FILTER_H
#define _SPEED_KALMAN_FILTER_H

#include "kalman_filter.h"
#include "robot_def.h"

#define VEL_PROCESS_NOISE 20.0f   // 速度过程噪声 20
#define ACC_PROCESS_NOISE 100.0f  // 加速度过程噪声 100

#define VEL_MEASURE_NOISE 100.0f  // 速度测量噪声 100
#define ACC_MEASURE_NOISE 0.01f  // 加速度测量噪声 0.01

void Speed_EstimateKF_Init(KalmanFilter_t *EstimateKF);
void Speed_KF_calc(KalmanFilter_t *Speed_EstimateKF, Leg *leg, float vel, float acc);

extern KalmanFilter_t Speed_EstimateKF;

extern float vel_acc[2];

#endif
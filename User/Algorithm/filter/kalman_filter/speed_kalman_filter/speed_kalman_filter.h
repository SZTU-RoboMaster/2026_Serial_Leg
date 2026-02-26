#ifndef _SPEED_KALMAN_FILTER_H
#define _SPEED_KALMAN_FILTER_H

#include "kalman_filter.h"
#include "robot_def.h"

#define VEL_PROCESS_NOISE 1000.0f
#define ACC_PROCESS_NOISE 1000.0f

#define VEL_MEASURE_NOISE 1500.0f
#define ACC_MEASURE_NOISE 900.0f

void Speed_EstimateKF_Init(KalmanFilter_t *EstimateKF);
void Speed_KF_calc(KalmanFilter_t *Speed_EstimateKF, Leg *leg, float vel, float acc);

extern KalmanFilter_t Speed_EstimateKF;

extern float vel_acc[2];

#endif
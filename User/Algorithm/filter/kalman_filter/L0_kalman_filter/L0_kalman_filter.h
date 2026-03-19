
#ifndef WHEEL_LEG_L0_KALMAN_FILTER_H
#define WHEEL_LEG_L0_KALMAN_FILTER_H

#include "kalman_filter.h"
#include "robot_def.h"

#define L0_PROCESS_NOISE 1000.0f
#define LO_dot_PROCESS_NOISE 1000.0f

#define L0_MEASURE_NOISE 1500.0f
#define L0_dot_MEASURE_NOISE 900.0f

void L0_EstimateKF_Init(KalmanFilter_t *EstimateKF);
void L0_KF_calc(KalmanFilter_t *L0_EstimateKF, Leg *leg, float L0, float L0_dot);

extern KalmanFilter_t L0_EstimateKF;

#endif //WHEEL_LEG_L0_KALMAN_FILTER_H
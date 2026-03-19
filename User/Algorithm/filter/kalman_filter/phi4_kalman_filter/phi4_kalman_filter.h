
#ifndef WHEEL_LEG_PHI4_KALMAN_FILTER_H
#define WHEEL_LEG_PHI4_KALMAN_FILTER_H

#include "kalman_filter.h"
#include "robot_def.h"

#define PHI4_PROCESS_NOISE 3000.0f
#define PHI4_dot_PROCESS_NOISE 1500.0f

#define PHI4_MEASURE_NOISE 300.0f
#define PHI4_dot_MEASURE_NOISE 150.0f


void PHI4_EstimateKF_Init(KalmanFilter_t *EstimateKF);
void PHI4_KF_calc(KalmanFilter_t *PHI4_EstimateKF, Leg *leg, float phi4, float phi4_dot);

extern KalmanFilter_t PHI4_EstimateKF;

#endif //WHEEL_LEG_PHI4_KALMAN_FILTER_H

#ifndef WHEEL_LEG_PHI0_KALMAN_FILTER_H
#define WHEEL_LEG_PHI0_KALMAN_FILTER_H

#include "kalman_filter.h"
#include "robot_def.h"

#define PHI0_PROCESS_NOISE 0.2f
#define PHI0_dot_PROCESS_NOISE 0.1f

#define PHI0_MEASURE_NOISE 3.5f
#define PHI0_dot_MEASURE_NOISE 3.0f

void PHI0_EstimateKF_Init(KalmanFilter_t *EstimateKF);
void PHI0_KF_calc(KalmanFilter_t *PHI0_EstimateKF, Leg *leg, float phi0, float phi0_dot);

extern KalmanFilter_t PHI0_EstimateKF;

#endif //WHEEL_LEG_PHI0_KALMAN_FILTER_H
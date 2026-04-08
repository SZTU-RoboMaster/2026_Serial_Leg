#ifndef _THETA_KALMAN_FILTER_H
#define _THETA_KALMAN_FILTER_H

#include "kalman_filter.h"
#include "robot_def.h"

// Q
#define THETA_PROCESS_NOISE 5.0f      // �ڽǹ�������
#define THETA_DOT_PROCESS_NOISE 5.0f  // �ڽǽ��ٶȹ�������

// R
#define THETA_MEASURE_NOISE 50.0f      // �ڽǲ�������  ����Ҫȷ���ǶȲ�ʧ��
#define THETA_DOT_MEASURE_NOISE 50.0f  // �ڽǽ��ٶȲ�������

void Theta_EstimateKF_Init(KalmanFilter_t *Theta_EstimateKF);

void theta_KF_calc(KalmanFilter_t *Theta_EstimateKF, Leg *leg, float theta, float theta_dot);

extern KalmanFilter_t Theta_EstimateKF_LegL;
extern KalmanFilter_t Theta_EstimateKF_LegR;

#endif
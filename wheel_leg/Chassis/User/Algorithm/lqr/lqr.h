#ifndef _LQR_H
#define _LQR_H

extern float wheel_K_L[6];
extern float joint_K_L[6];
extern float wheel_K_R[6];
extern float joint_K_R[6];

// ÄâºÏÏµÊý
extern float wheel_fitting_factor[6][4];
extern float joint_fitting_factor[6][4];

void chassis_K_matrix_fitting(float L0, float K[6], const float KL[6][4]);

#endif
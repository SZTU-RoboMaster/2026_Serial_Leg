#ifndef VMC_H
#define VMC_H

#include "robot_def.h"

void vmc_phi_update(Leg *leg_L, Leg *leg_R);
float cal_leg_theta(float phi0, float phi);
void vmc_forward_dynamics(VMC *vmc, const ChassisPhysicalConfig *physical_config);
void vmc_calc(void);

#endif
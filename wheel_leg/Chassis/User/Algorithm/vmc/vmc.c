#include <stdbool.h>
#include <math.h>
#include <stdio.h>

#include "robot_def.h"
#include "vmc.h"
#include "user_lib.h"
#include "joint.h"
#include "moving_filter.h"
#include "vofa.h"

extern Chassis chassis;
extern ChassisPhysicalConfig chassis_physical_config;

/*               正方向
 *    phi4                      phi4
 *
 *    phi1                      phi1
 */

void vmc_phi_update(Leg *leg_L, Leg *leg_R) {

    float LF_joint_pos = (get_joint_motors() + 0)->pos_r;
    float LB_joint_pos = (get_joint_motors() + 1)->pos_r;
    float RF_joint_pos = (get_joint_motors() + 2)->pos_r;
    float RB_joint_pos = (get_joint_motors() + 3)->pos_r;

    leg_L->vmc.forward_kinematics.fk_phi.phi1 = LF_joint_pos;
    leg_L->vmc.forward_kinematics.fk_phi.phi4 = LB_joint_pos;
    leg_R->vmc.forward_kinematics.fk_phi.phi1 = -RF_joint_pos;
    leg_R->vmc.forward_kinematics.fk_phi.phi4 = -RB_joint_pos;

}

/** 计算状态变量theta **/
float cal_leg_theta(float phi0, float phi) {

    float theta = phi0 - PI/2 - phi;

    return theta;
}

// vmc正运动学解算
static void forward_kinematics(Leg* leg_L, Leg* leg_R, ChassisPhysicalConfig *physical_config) {
    /***LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L LEG_L***/

    /** phi2 **/
    leg_L->vmc.forward_kinematics.fk_point_coordinates.b_x = physical_config->l1 * cosf(leg_L->vmc.forward_kinematics.fk_phi.phi1);
    leg_L->vmc.forward_kinematics.fk_point_coordinates.b_y = physical_config->l1 * sinf(leg_L->vmc.forward_kinematics.fk_phi.phi1);
    leg_L->vmc.forward_kinematics.fk_point_coordinates.d_x = physical_config->l5 + physical_config->l4 * cosf(leg_L->vmc.forward_kinematics.fk_phi.phi4);
    leg_L->vmc.forward_kinematics.fk_point_coordinates.d_y = physical_config->l4 * sinf(leg_L->vmc.forward_kinematics.fk_phi.phi4);

    float L_A0 = 2.0f * physical_config->l2 * (leg_L->vmc.forward_kinematics.fk_point_coordinates.d_x - leg_L->vmc.forward_kinematics.fk_point_coordinates.b_x);
    float L_B0 = 2.0f * physical_config->l2 * (leg_L->vmc.forward_kinematics.fk_point_coordinates.d_y - leg_L->vmc.forward_kinematics.fk_point_coordinates.b_y);
    float L_BD_sq =  (leg_L->vmc.forward_kinematics.fk_point_coordinates.d_x - leg_L->vmc.forward_kinematics.fk_point_coordinates.b_x) * (leg_L->vmc.forward_kinematics.fk_point_coordinates.d_x - leg_L->vmc.forward_kinematics.fk_point_coordinates.b_x)
                     + (leg_L->vmc.forward_kinematics.fk_point_coordinates.d_y - leg_L->vmc.forward_kinematics.fk_point_coordinates.b_y) * (leg_L->vmc.forward_kinematics.fk_point_coordinates.d_y - leg_L->vmc.forward_kinematics.fk_point_coordinates.b_y);
    float L_C0 = physical_config->l2 * physical_config->l2 + L_BD_sq - physical_config->l3 * physical_config->l3;

    float temp = L_A0 * L_A0 + L_B0 * L_B0 - L_C0 * L_C0;
    float y = L_B0 + sqrtf(ABS(temp));
    float x = L_A0 + L_C0;
    leg_L->vmc.forward_kinematics.fk_phi.phi2 = 2.0f * atan2f(y, x);

    /** C点坐标 **/
    leg_L->vmc.forward_kinematics.fk_point_coordinates.c_x = physical_config->l1 * cosf(leg_L->vmc.forward_kinematics.fk_phi.phi1) + physical_config->l2 * cosf(leg_L->vmc.forward_kinematics.fk_phi.phi2);
    leg_L->vmc.forward_kinematics.fk_point_coordinates.c_y = physical_config->l1 * sinf(leg_L->vmc.forward_kinematics.fk_phi.phi1) + physical_config->l2 * sinf(leg_L->vmc.forward_kinematics.fk_phi.phi2);

    /** phi3 **/
    y = leg_L->vmc.forward_kinematics.fk_point_coordinates.c_y - leg_L->vmc.forward_kinematics.fk_point_coordinates.d_y;
    x = leg_L->vmc.forward_kinematics.fk_point_coordinates.c_x - leg_L->vmc.forward_kinematics.fk_point_coordinates.d_x;
    leg_L->vmc.forward_kinematics.fk_phi.phi3 = atan2f(y, x);

    /** 腿长(L0) **/
    temp =  (leg_L->vmc.forward_kinematics.fk_point_coordinates.c_x - physical_config->l5 * 0.5f) * (leg_L->vmc.forward_kinematics.fk_point_coordinates.c_x - physical_config->l5 * 0.5f)
            + leg_L->vmc.forward_kinematics.fk_point_coordinates.c_y * leg_L->vmc.forward_kinematics.fk_point_coordinates.c_y;
    leg_L->vmc.forward_kinematics.fk_L0.L0_last = leg_L->vmc.forward_kinematics.fk_L0.L0;
    leg_L->vmc.forward_kinematics.fk_L0.L0 = sqrtf(ABS(temp));
    leg_L->vmc.forward_kinematics.fk_L0.L0_dot_last = leg_L->vmc.forward_kinematics.fk_L0.L0_dot;
    leg_L->vmc.forward_kinematics.fk_L0.L0_dot =
            (leg_L->vmc.forward_kinematics.fk_L0.L0 - leg_L->vmc.forward_kinematics.fk_L0.L0_last)
            / (CHASSIS_PERIOD * 0.001f);
    leg_L->vmc.forward_kinematics.fk_L0.L0_ddot =
            (leg_L->vmc.forward_kinematics.fk_L0.L0_dot - leg_L->vmc.forward_kinematics.fk_L0.L0_dot_last)
            / (CHASSIS_PERIOD * 0.001f);

    /** 腿摆角（phi0） **/
    leg_L->vmc.forward_kinematics.fk_phi.last_phi0 = leg_L->vmc.forward_kinematics.fk_phi.phi0;
    y = leg_L->vmc.forward_kinematics.fk_point_coordinates.c_y;
    x = leg_L->vmc.forward_kinematics.fk_point_coordinates.c_x - physical_config->l5 * 0.5f;
    leg_L->vmc.forward_kinematics.fk_phi.phi0 = atan2f(y, x);
    leg_L->vmc.forward_kinematics.fk_phi.last_d_phi0 = leg_L->vmc.forward_kinematics.fk_phi.d_phi0;
    leg_L->vmc.forward_kinematics.fk_phi.d_phi0 =  (leg_L->vmc.forward_kinematics.fk_phi.phi0 - leg_L->vmc.forward_kinematics.fk_phi.last_phi0)
                                                   / (CHASSIS_PERIOD * 0.001f);
    leg_L->vmc.forward_kinematics.fk_phi.dd_phi0 =  (leg_L->vmc.forward_kinematics.fk_phi.d_phi0 - leg_L->vmc.forward_kinematics.fk_phi.last_d_phi0)
                                                    / (CHASSIS_PERIOD * 0.001f);

    /** ? **/
    leg_L->vmc.forward_kinematics.d_alpha = 0.0f - leg_L->vmc.forward_kinematics.fk_phi.d_phi0;

    /***LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R***/

    leg_R->vmc.forward_kinematics.fk_point_coordinates.b_x = physical_config->l1 * cosf(leg_R->vmc.forward_kinematics.fk_phi.phi1);
    leg_R->vmc.forward_kinematics.fk_point_coordinates.b_y = physical_config->l1 * sinf(leg_R->vmc.forward_kinematics.fk_phi.phi1);
    leg_R->vmc.forward_kinematics.fk_point_coordinates.d_x = physical_config->l5 + physical_config->l4 * cosf(leg_R->vmc.forward_kinematics.fk_phi.phi4);
    leg_R->vmc.forward_kinematics.fk_point_coordinates.d_y = physical_config->l4 * sinf(leg_R->vmc.forward_kinematics.fk_phi.phi4);

    float R_A0 = 2.0f * physical_config->l2 * (leg_R->vmc.forward_kinematics.fk_point_coordinates.d_x - leg_R->vmc.forward_kinematics.fk_point_coordinates.b_x);
    float R_B0 = 2.0f * physical_config->l2 * (leg_R->vmc.forward_kinematics.fk_point_coordinates.d_y - leg_R->vmc.forward_kinematics.fk_point_coordinates.b_y);
    float R_BD_sq =  (leg_R->vmc.forward_kinematics.fk_point_coordinates.d_x - leg_R->vmc.forward_kinematics.fk_point_coordinates.b_x) * (leg_R->vmc.forward_kinematics.fk_point_coordinates.d_x - leg_R->vmc.forward_kinematics.fk_point_coordinates.b_x)
                     + (leg_R->vmc.forward_kinematics.fk_point_coordinates.d_y - leg_R->vmc.forward_kinematics.fk_point_coordinates.b_y) * (leg_R->vmc.forward_kinematics.fk_point_coordinates.d_y - leg_R->vmc.forward_kinematics.fk_point_coordinates.b_y);
    float R_C0 = physical_config->l2 * physical_config->l2 + R_BD_sq - physical_config->l3 * physical_config->l3;

    temp = R_A0 * R_A0 + R_B0 * R_B0 - R_C0 * R_C0;
    y = R_B0 + sqrtf(ABS(temp));
    x = R_A0 + R_C0;
    leg_R->vmc.forward_kinematics.fk_phi.phi2 = 2.0f * atan2f(y, x);

    leg_R->vmc.forward_kinematics.fk_point_coordinates.c_x = physical_config->l1 * cosf(leg_R->vmc.forward_kinematics.fk_phi.phi1) + physical_config->l2 * cosf(leg_R->vmc.forward_kinematics.fk_phi.phi2);
    leg_R->vmc.forward_kinematics.fk_point_coordinates.c_y = physical_config->l1 * sinf(leg_R->vmc.forward_kinematics.fk_phi.phi1) + physical_config->l2 * sinf(leg_R->vmc.forward_kinematics.fk_phi.phi2);
    y = leg_R->vmc.forward_kinematics.fk_point_coordinates.c_y - leg_R->vmc.forward_kinematics.fk_point_coordinates.d_y;
    x = leg_R->vmc.forward_kinematics.fk_point_coordinates.c_x - leg_R->vmc.forward_kinematics.fk_point_coordinates.d_x;
    leg_R->vmc.forward_kinematics.fk_phi.phi3 = atan2f(y, x);

    temp =  (leg_R->vmc.forward_kinematics.fk_point_coordinates.c_x - physical_config->l5 * 0.5f) * (leg_R->vmc.forward_kinematics.fk_point_coordinates.c_x - physical_config->l5 * 0.5f)
            + leg_R->vmc.forward_kinematics.fk_point_coordinates.c_y * leg_R->vmc.forward_kinematics.fk_point_coordinates.c_y;
    leg_R->vmc.forward_kinematics.fk_L0.L0_last = leg_R->vmc.forward_kinematics.fk_L0.L0;
    leg_R->vmc.forward_kinematics.fk_L0.L0 = sqrtf(ABS(temp));
    leg_R->vmc.forward_kinematics.fk_L0.L0_dot_last = leg_R->vmc.forward_kinematics.fk_L0.L0_dot;
    leg_R->vmc.forward_kinematics.fk_L0.L0_dot = (leg_R->vmc.forward_kinematics.fk_L0.L0 - leg_R->vmc.forward_kinematics.fk_L0.L0_last) / (CHASSIS_PERIOD * 0.001f);
    leg_R->vmc.forward_kinematics.fk_L0.L0_ddot = (leg_R->vmc.forward_kinematics.fk_L0.L0_dot - leg_R->vmc.forward_kinematics.fk_L0.L0_dot_last) / (CHASSIS_PERIOD * 0.001f);

    leg_R->vmc.forward_kinematics.fk_phi.last_phi0 = leg_R->vmc.forward_kinematics.fk_phi.phi0;
    y = leg_R->vmc.forward_kinematics.fk_point_coordinates.c_y;
    x = leg_R->vmc.forward_kinematics.fk_point_coordinates.c_x - physical_config->l5 * 0.5f;


    leg_R->vmc.forward_kinematics.fk_phi.phi0 = atan2f(y, x);
    leg_R->vmc.forward_kinematics.fk_phi.last_d_phi0 = leg_R->vmc.forward_kinematics.fk_phi.d_phi0;
    leg_R->vmc.forward_kinematics.fk_phi.d_phi0 =  (leg_R->vmc.forward_kinematics.fk_phi.phi0 - leg_R->vmc.forward_kinematics.fk_phi.last_phi0) / (CHASSIS_PERIOD * 0.001f);
    leg_R->vmc.forward_kinematics.fk_phi.dd_phi0 =  (leg_R->vmc.forward_kinematics.fk_phi.d_phi0 - leg_R->vmc.forward_kinematics.fk_phi.last_d_phi0) / (CHASSIS_PERIOD * 0.001f);

    leg_R->vmc.forward_kinematics.d_alpha = 0.0f - leg_R->vmc.forward_kinematics.fk_phi.d_phi0;

    /** 防劈叉 **/
    chassis.last_phi0_error = chassis.phi0_error;
    chassis.phi0_error = leg_L->vmc.forward_kinematics.fk_phi.phi0 - leg_R->vmc.forward_kinematics.fk_phi.phi0;
    chassis.d_phi0_error = (chassis.phi0_error - chassis.last_phi0_error) / (CHASSIS_PERIOD * 0.001f);

}

// vmc正动力学解算
void vmc_forward_dynamics(VMC *vmc, const ChassisPhysicalConfig *physical_config) {

    if (vmc == NULL)
    {
        return;
    }

    vmc->forward_kinematics.J_F_to_T.E.x1_1 =
            physical_config->l1 * cosf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi3) * sinf(vmc->forward_kinematics.fk_phi.phi1 - vmc->forward_kinematics.fk_phi.phi2)
            / (vmc->forward_kinematics.fk_L0.L0 * sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi2));

    vmc->forward_kinematics.J_F_to_T.E.x1_2 =
            physical_config->l1 * sinf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi3) * sinf(vmc->forward_kinematics.fk_phi.phi1 - vmc->forward_kinematics.fk_phi.phi2)
            / sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi2);

    vmc->forward_kinematics.J_F_to_T.E.x2_1 =
            physical_config->l4 * cosf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi2) * sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi4)
            / (vmc->forward_kinematics.fk_L0.L0 * sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi2));

    vmc->forward_kinematics.J_F_to_T.E.x2_2 = physical_config->l4 * sinf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi2) * sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi4)
                                              / sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi2);

    Matrix_multiply(2, 2, vmc->forward_kinematics.J_F_to_T.array,
                    2, 1, vmc->forward_kinematics.Fxy_set_point.array,
                    vmc->forward_kinematics.T1_T4_set_point.array);
}


// 逆解算腿长变化速度、摆角变化速度
static void vmc_inverse_kinematics(VMC *vmc,
                                   float w1,
                                   float w4,
                                   ChassisPhysicalConfig *chassis_physical_config) {
    if (vmc == NULL) {
        return;
    }
    vmc->inverse_kinematics.W_fdb.E.w1_fdb = w1;
    vmc->inverse_kinematics.W_fdb.E.w4_fdb = w4;

    vmc->inverse_kinematics.J_w_to_v.E.x1_1 =  -chassis_physical_config->l1 * sinf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi3) * sinf(vmc->forward_kinematics.fk_phi.phi1 - vmc->forward_kinematics.fk_phi.phi2)
                                               / sinf(vmc->forward_kinematics.fk_phi.phi2 - vmc->forward_kinematics.fk_phi.phi3);


    vmc->inverse_kinematics.J_w_to_v.E.x1_2 = -chassis_physical_config->l4 * sinf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi2) * sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi4)
                                              / sinf(vmc->forward_kinematics.fk_phi.phi2 - vmc->forward_kinematics.fk_phi.phi3);


    vmc->inverse_kinematics.J_w_to_v.E.x2_1 = -chassis_physical_config->l1 * cosf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi3) * sinf(vmc->forward_kinematics.fk_phi.phi1 - vmc->forward_kinematics.fk_phi.phi2)
                                              / (vmc->forward_kinematics.fk_L0.L0 * sinf(vmc->forward_kinematics.fk_phi.phi2 - vmc->forward_kinematics.fk_phi.phi3));


    vmc->inverse_kinematics.J_w_to_v.E.x2_2 = -chassis_physical_config->l4 * cosf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi2) * sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi4)
                                              / (vmc->forward_kinematics.fk_L0.L0 * sinf(vmc->forward_kinematics.fk_phi.phi2 - vmc->forward_kinematics.fk_phi.phi3));

    vmc->inverse_kinematics.V_fdb.E.last_d_L0_fdb = vmc->inverse_kinematics.V_fdb.E.d_L0_fdb;

    Matrix_multiply(2, 2, vmc->inverse_kinematics.J_w_to_v.array,
                    2, 1, vmc->inverse_kinematics.W_fdb.array,
                    vmc->inverse_kinematics.V_fdb.array);


    vmc->inverse_kinematics.V_fdb.E.dd_L0_fdb = (vmc->inverse_kinematics.V_fdb.E.d_L0_fdb - vmc->inverse_kinematics.V_fdb.E.last_d_L0_fdb) / (CHASSIS_PERIOD * 0.001f);
}

// 逆解算出虚拟力矩和沿腿方向支持力
static void vmc_inverse_dynamics(VMC *vmc,
                                 float T1, // phi1
                                 float T4, // phi4
                                 ChassisPhysicalConfig *chassis_physical_config) {
    if (vmc == NULL) {
        return;
    }
    vmc->inverse_kinematics.T1_T4_fdb.E.T1_fdb = T1;
    vmc->inverse_kinematics.T1_T4_fdb.E.T4_fdb = T4;

    vmc->inverse_kinematics.J_T_to_F.E.x1_1 =
            vmc->forward_kinematics.fk_L0.L0 * sinf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi2)
            / (chassis_physical_config->l1
               * sinf(vmc->forward_kinematics.fk_phi.phi1 - vmc->forward_kinematics.fk_phi.phi2));

    vmc->inverse_kinematics.J_T_to_F.E.x1_2 =
            vmc->forward_kinematics.fk_L0.L0 * sinf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi3)
            / (chassis_physical_config->l4
               * sinf(vmc->forward_kinematics.fk_phi.phi4 - vmc->forward_kinematics.fk_phi.phi3));

    vmc->inverse_kinematics.J_T_to_F.E.x2_1 = cosf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi2)
                                              / (chassis_physical_config->l1 * sinf(vmc->forward_kinematics.fk_phi.phi2 - vmc->forward_kinematics.fk_phi.phi1));

    vmc->inverse_kinematics.J_T_to_F.E.x2_2 = cosf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi3)
                                              / (chassis_physical_config->l4 * sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi4));


    Matrix_multiply(2, 2, vmc->inverse_kinematics.J_T_to_F.array,
                    2, 1, vmc->inverse_kinematics.T1_T4_fdb.array,
                    vmc->inverse_kinematics.Fxy_fdb.array);
}

// 计算竖直方向支持力
static void fn_cal(Leg *leg, float body_az, ChassisPhysicalConfig *chassis_physical_config) {

    if (leg == NULL) {
        return;
    }

    // 用逆解算的数据计算
    float P = leg->vmc.inverse_kinematics.Fxy_fdb.E.Fy_fdb * cosf(leg->state_variable_feedback.theta) // 没准有个负号
              + leg->vmc.inverse_kinematics.Fxy_fdb.E.Tp_fdb * sinf(leg->state_variable_feedback.theta) / leg->vmc.forward_kinematics.fk_L0.L0;

    float wheel_az = body_az - leg->vmc.inverse_kinematics.V_fdb.E.dd_L0_fdb * cosf(leg->state_variable_feedback.theta)
                     + 2.0f * leg->vmc.inverse_kinematics.V_fdb.E.d_L0_fdb * leg->state_variable_feedback.theta_dot
                       * sinf(leg->state_variable_feedback.theta)
                     + leg->vmc.forward_kinematics.fk_L0.L0 * leg->state_variable_feedback.theta_ddot
                       * sinf(leg->state_variable_feedback.theta)
                     + leg->vmc.forward_kinematics.fk_L0.L0 * leg->state_variable_feedback.theta_dot
                       * leg->state_variable_feedback.theta_dot * cosf(leg->state_variable_feedback.theta);


    leg->Fn = P + chassis_physical_config->wheel_weight * (GRAVITY + wheel_az);

}

/*******************************************************************************
 *                                     VMC                                     *
 *******************************************************************************/
void vmc_calc(void) {

    // 更新phi1 phi4
    vmc_phi_update(&chassis.leg_L, &chassis.leg_R);

    // VMC 正运动学解算
    forward_kinematics(&chassis.leg_L, &chassis.leg_R, &chassis_physical_config);
}
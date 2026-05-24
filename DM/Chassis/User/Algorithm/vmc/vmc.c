#include <stdbool.h>
#include <math.h>
#include <stdio.h>

#include "robot_def.h"
#include "vmc.h"
#include "user_lib.h"
#include "joint.h"
#include "vofa.h"

extern Chassis chassis;
extern ChassisPhysicalConfig chassis_physical_config;

/*               正方向
 *    phi4                      phi4
 *
 *    phi1                      phi1
 */

// 归一化角度到 [-PI,PI]
static float wrap_pi(float angle)
{
    while (angle > PI) {
        angle -= 2.0f * PI;
    }
    while (angle < -PI) {
        angle += 2.0f * PI;
    }
    return angle;
}

static float vmc_angle_diff_rad(float target, float current)
{
    float diff = target - current;
    return atan2f(sinf(diff), cosf(diff));
}

static float vmc_calc_phi3(float phi1, float phi4, const ChassisPhysicalConfig *physical_config)
{
    float b_x = physical_config->l1 * cosf(phi1);
    float b_y = physical_config->l1 * sinf(phi1);
    float d_x = physical_config->l5 + physical_config->l4 * cosf(phi4);
    float d_y = physical_config->l4 * sinf(phi4);

    float A0 = 2.0f * physical_config->l2 * (d_x - b_x);
    float B0 = 2.0f * physical_config->l2 * (d_y - b_y);
    float BD_sq = (d_x - b_x) * (d_x - b_x) + (d_y - b_y) * (d_y - b_y);
    float C0 = physical_config->l2 * physical_config->l2 + BD_sq - physical_config->l3 * physical_config->l3;
    float phi2 = 2.0f * atan2f(B0 + sqrtf(ABS(A0 * A0 + B0 * B0 - C0 * C0)), A0 + C0);

    float c_x = physical_config->l1 * cosf(phi1) + physical_config->l2 * cosf(phi2);
    float c_y = physical_config->l1 * sinf(phi1) + physical_config->l2 * sinf(phi2);

    return atan2f(c_y - d_y, c_x - d_x);
}
void vmc_phi_update(Leg *leg_L, Leg *leg_R) {

    // 角度
    float LF_joint_pos = (get_joint_motors() + 0)->pos_r;
    float LB_joint_pos = (get_joint_motors() + 1)->pos_r;
    float RF_joint_pos = (get_joint_motors() + 2)->pos_r;
    float RB_joint_pos = (get_joint_motors() + 3)->pos_r;

//    USART_Vofa_Justfloat_Transmit(LF_joint_pos,LB_joint_pos);
//    USART_Vofa_Justfloat_Transmit(RF_joint_pos,RB_joint_pos);


    leg_L->vmc.forward_kinematics.fk_phi.phi1 = wrap_pi(LF_joint_pos + PHI1_OFFSET);
    leg_L->vmc.forward_kinematics.fk_phi.phi4 = wrap_pi(LB_joint_pos + PHI4_OFFSET);
    leg_R->vmc.forward_kinematics.fk_phi.phi1 = wrap_pi(-RF_joint_pos + PHI1_OFFSET);
    leg_R->vmc.forward_kinematics.fk_phi.phi4 = wrap_pi(-RB_joint_pos + PHI4_OFFSET);

//    USART_Vofa_Justfloat_Transmit(leg_L->vmc.forward_kinematics.fk_phi.phi4,leg_L->vmc.forward_kinematics.fk_phi.phi1);
//    USART_Vofa_Justfloat_Transmit(leg_R->vmc.forward_kinematics.fk_phi.phi4,leg_R->vmc.forward_kinematics.fk_phi.phi1);

    // 角速度
    float LF_joint_speed = (get_joint_motors() + 0)->angular_vel;
    float LB_joint_speed = (get_joint_motors() + 1)->angular_vel;
    float RF_joint_speed = (get_joint_motors() + 2)->angular_vel;
    float RB_joint_speed = (get_joint_motors() + 3)->angular_vel;

    leg_L->vmc.forward_kinematics.fk_phi.phi1_dot = LF_joint_speed;
    leg_L->vmc.forward_kinematics.fk_phi.phi4_dot = LB_joint_speed;
    leg_R->vmc.forward_kinematics.fk_phi.phi1_dot = -RF_joint_speed;
    leg_R->vmc.forward_kinematics.fk_phi.phi4_dot = -RB_joint_speed;

//    USART_Vofa_Justfloat_Transmit(leg_L->vmc.forward_kinematics.fk_phi.phi4_dot,leg_L->vmc.forward_kinematics.fk_phi.phi1_dot);
//    USART_Vofa_Justfloat_Transmit(leg_R->vmc.forward_kinematics.fk_phi.phi4_dot,leg_R->vmc.forward_kinematics.fk_phi.phi1_dot);
}

// vmc正运动学解算
static void forward_kinematics(Leg *leg, ChassisPhysicalConfig *physical_config, float dt) {

    // 中间变量
    float temp = 0;
    float temp_y = 0;
    float temp_x = 0;

    /** 1、phi2 **/
    leg->vmc.forward_kinematics.fk_point_coordinates.b_x =
            physical_config->l1 * cosf(leg->vmc.forward_kinematics.fk_phi.phi1);
    leg->vmc.forward_kinematics.fk_point_coordinates.b_y =
            physical_config->l1 * sinf(leg->vmc.forward_kinematics.fk_phi.phi1);
    leg->vmc.forward_kinematics.fk_point_coordinates.d_x =
            physical_config->l5 + physical_config->l4 * cosf(leg->vmc.forward_kinematics.fk_phi.phi4);
    leg->vmc.forward_kinematics.fk_point_coordinates.d_y =
            physical_config->l4 * sinf(leg->vmc.forward_kinematics.fk_phi.phi4);

    float A0 = 2.0f * physical_config->l2 * (leg->vmc.forward_kinematics.fk_point_coordinates.d_x -
                                             leg->vmc.forward_kinematics.fk_point_coordinates.b_x);
    float B0 = 2.0f * physical_config->l2 * (leg->vmc.forward_kinematics.fk_point_coordinates.d_y -
                                             leg->vmc.forward_kinematics.fk_point_coordinates.b_y);

    float BD_sq =  (leg->vmc.forward_kinematics.fk_point_coordinates.d_x - leg->vmc.forward_kinematics.fk_point_coordinates.b_x)
                 * (leg->vmc.forward_kinematics.fk_point_coordinates.d_x - leg->vmc.forward_kinematics.fk_point_coordinates.b_x)
                 + (leg->vmc.forward_kinematics.fk_point_coordinates.d_y - leg->vmc.forward_kinematics.fk_point_coordinates.b_y)
                 * (leg->vmc.forward_kinematics.fk_point_coordinates.d_y - leg->vmc.forward_kinematics.fk_point_coordinates.b_y);

    float C0 = physical_config->l2 * physical_config->l2 + BD_sq - physical_config->l3 * physical_config->l3;

    temp = A0 * A0 + B0 * B0 - C0 * C0;
    temp_y = B0 + sqrtf(ABS(temp));
    temp_x = A0 + C0;

    leg->vmc.forward_kinematics.fk_phi.phi2 = 2.0f * atan2f(temp_y, temp_x);

    /** 2、C点直角坐标 **/
    leg->vmc.forward_kinematics.fk_point_coordinates.c_x =
            physical_config->l1 * cosf(leg->vmc.forward_kinematics.fk_phi.phi1) +
            physical_config->l2 * cosf(leg->vmc.forward_kinematics.fk_phi.phi2);
    leg->vmc.forward_kinematics.fk_point_coordinates.c_y =
            physical_config->l1 * sinf(leg->vmc.forward_kinematics.fk_phi.phi1) +
            physical_config->l2 * sinf(leg->vmc.forward_kinematics.fk_phi.phi2);

    /** 3、phi3 **/
    temp_y = leg->vmc.forward_kinematics.fk_point_coordinates.c_y - leg->vmc.forward_kinematics.fk_point_coordinates.d_y;
    temp_x = leg->vmc.forward_kinematics.fk_point_coordinates.c_x - leg->vmc.forward_kinematics.fk_point_coordinates.d_x;
    leg->vmc.forward_kinematics.fk_phi.phi3 = atan2f(temp_y, temp_x);

    /** 3.1 phi3_dot 前向差分 **/
    // 预测步长
    float phi3_predict_dt = dt;

    if (phi3_predict_dt <= 0.000001f)
    {
        phi3_predict_dt = CHASSIS_PERIOD * 0.001f;
    }

    float phi1_pred = leg->vmc.forward_kinematics.fk_phi.phi1 +
                      leg->vmc.forward_kinematics.fk_phi.phi1_dot * phi3_predict_dt;

    float phi4_pred = leg->vmc.forward_kinematics.fk_phi.phi4 +
                      leg->vmc.forward_kinematics.fk_phi.phi4_dot * phi3_predict_dt;

    float phi3_pred = vmc_calc_phi3(phi1_pred, phi4_pred, physical_config);

    leg->vmc.forward_kinematics.fk_phi.phi3_dot =
            vmc_angle_diff_rad(phi3_pred, leg->vmc.forward_kinematics.fk_phi.phi3) / phi3_predict_dt;

    /** 4、腿长(L0) **/
    temp = (leg->vmc.forward_kinematics.fk_point_coordinates.c_x - physical_config->l5 * 0.5f) *
           (leg->vmc.forward_kinematics.fk_point_coordinates.c_x - physical_config->l5 * 0.5f)
           + leg->vmc.forward_kinematics.fk_point_coordinates.c_y *
             leg->vmc.forward_kinematics.fk_point_coordinates.c_y;
    leg->vmc.forward_kinematics.fk_L0.L0 = sqrtf(ABS(temp));

    /** 5、精确求解L0_dot **/
    // 求A1
    temp_y = physical_config->l1 * leg->vmc.forward_kinematics.fk_phi.phi1_dot *
             sinf(leg->vmc.forward_kinematics.fk_phi.phi1 - leg->vmc.forward_kinematics.fk_phi.phi3) +
             physical_config->l4 * leg->vmc.forward_kinematics.fk_phi.phi4_dot *
             sinf(leg->vmc.forward_kinematics.fk_phi.phi3 - leg->vmc.forward_kinematics.fk_phi.phi4);

    temp_x = sinf(leg->vmc.forward_kinematics.fk_phi.phi3 - leg->vmc.forward_kinematics.fk_phi.phi2);

    float A1 = temp_y / temp_x;

    float Xb_dot = -physical_config->l1 * leg->vmc.forward_kinematics.fk_phi.phi1_dot *
                   sinf(leg->vmc.forward_kinematics.fk_phi.phi1);

    float Yb_dot = physical_config->l1 * leg->vmc.forward_kinematics.fk_phi.phi1_dot *
                   cosf(leg->vmc.forward_kinematics.fk_phi.phi1);

    // 求L0_dot
    temp_y = leg->vmc.forward_kinematics.fk_point_coordinates.c_y *
             (Yb_dot + A1 * cosf(leg->vmc.forward_kinematics.fk_phi.phi2)) +
             (leg->vmc.forward_kinematics.fk_point_coordinates.c_x - (physical_config->l5 / 2.0f)) *
             (Xb_dot - A1 * sinf(leg->vmc.forward_kinematics.fk_phi.phi2));
    temp_x = leg->vmc.forward_kinematics.fk_L0.L0;

    leg->vmc.forward_kinematics.fk_L0.L0_dot_last = leg->vmc.forward_kinematics.fk_L0.L0_dot;
    leg->vmc.forward_kinematics.fk_L0.L0_dot = temp_y / temp_x;

    /**  6、腿长L0_ddot差分求解 应加低通滤波 **/
    leg->vmc.forward_kinematics.fk_L0.L0_ddot =
            (leg->vmc.forward_kinematics.fk_L0.L0_dot - leg->vmc.forward_kinematics.fk_L0.L0_dot_last)
            / dt;

    /**  7、腿摆角（phi0） **/
    temp_y = leg->vmc.forward_kinematics.fk_point_coordinates.c_y;
    temp_x = leg->vmc.forward_kinematics.fk_point_coordinates.c_x - physical_config->l5 * 0.5f;
    leg->vmc.forward_kinematics.fk_phi.phi0 = atan2f(temp_y, temp_x);

    /** 8、精确求解phi0_dot **/
    // 求phi0_dot
    temp_y = (leg->vmc.forward_kinematics.fk_point_coordinates.c_x - (physical_config->l5 / 2.0f)) *
             (Yb_dot + A1 * cosf(leg->vmc.forward_kinematics.fk_phi.phi2)) -
             leg->vmc.forward_kinematics.fk_point_coordinates.c_y *
             (Xb_dot - A1 * sinf(leg->vmc.forward_kinematics.fk_phi.phi2));
    temp_x = leg->vmc.forward_kinematics.fk_L0.L0 * leg->vmc.forward_kinematics.fk_L0.L0;

    leg->vmc.forward_kinematics.fk_phi.last_d_phi0 = leg->vmc.forward_kinematics.fk_phi.d_phi0;
    leg->vmc.forward_kinematics.fk_phi.d_phi0 = temp_y / temp_x;

    /**  9、phi0_ddot 差分求解 **/
    leg->vmc.forward_kinematics.fk_phi.dd_phi0 =
            (leg->vmc.forward_kinematics.fk_phi.d_phi0 - leg->vmc.forward_kinematics.fk_phi.last_d_phi0)
            / dt;

}

static void leg_coordinate_handle(float dt) {
    /** 双腿协调 **/
    chassis.last_phi0_error = chassis.phi0_error;
    chassis.phi0_error = chassis.leg_L.vmc.forward_kinematics.fk_phi.phi0 - chassis.leg_R.vmc.forward_kinematics.fk_phi.phi0;
    chassis.d_phi0_error = (chassis.phi0_error - chassis.last_phi0_error) / dt;

}

// vmc正动力学解算
void vmc_forward_dynamics(VMC *vmc, const ChassisPhysicalConfig *physical_config) {

    if (vmc == NULL) {
        return;
    }

    vmc->forward_kinematics.J_F_to_T.E.x1_1 =
            physical_config->l1 * cosf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi3) *
            sinf(vmc->forward_kinematics.fk_phi.phi1 - vmc->forward_kinematics.fk_phi.phi2)
            / (vmc->forward_kinematics.fk_L0.L0 *
               sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi2));

    vmc->forward_kinematics.J_F_to_T.E.x1_2 =
            physical_config->l1 * sinf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi3) *
            sinf(vmc->forward_kinematics.fk_phi.phi1 - vmc->forward_kinematics.fk_phi.phi2)
            / sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi2);

    vmc->forward_kinematics.J_F_to_T.E.x2_1 =
            physical_config->l4 * cosf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi2) *
            sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi4)
            / (vmc->forward_kinematics.fk_L0.L0 *
               sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi2));

    vmc->forward_kinematics.J_F_to_T.E.x2_2 =
            physical_config->l4 * sinf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi2) *
            sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi4)
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

    vmc->inverse_kinematics.J_w_to_v.E.x1_1 = -chassis_physical_config->l1 * sinf(vmc->forward_kinematics.fk_phi.phi0 -
                                                                                  vmc->forward_kinematics.fk_phi.phi3) *
                                              sinf(vmc->forward_kinematics.fk_phi.phi1 -
                                                   vmc->forward_kinematics.fk_phi.phi2)
                                              / sinf(vmc->forward_kinematics.fk_phi.phi2 -
                                                     vmc->forward_kinematics.fk_phi.phi3);


    vmc->inverse_kinematics.J_w_to_v.E.x1_2 = -chassis_physical_config->l4 * sinf(vmc->forward_kinematics.fk_phi.phi0 -
                                                                                  vmc->forward_kinematics.fk_phi.phi2) *
                                              sinf(vmc->forward_kinematics.fk_phi.phi3 -
                                                   vmc->forward_kinematics.fk_phi.phi4)
                                              / sinf(vmc->forward_kinematics.fk_phi.phi2 -
                                                     vmc->forward_kinematics.fk_phi.phi3);


    vmc->inverse_kinematics.J_w_to_v.E.x2_1 = -chassis_physical_config->l1 * cosf(vmc->forward_kinematics.fk_phi.phi0 -
                                                                                  vmc->forward_kinematics.fk_phi.phi3) *
                                              sinf(vmc->forward_kinematics.fk_phi.phi1 -
                                                   vmc->forward_kinematics.fk_phi.phi2)
                                              / (vmc->forward_kinematics.fk_L0.L0 *
                                                 sinf(vmc->forward_kinematics.fk_phi.phi2 -
                                                      vmc->forward_kinematics.fk_phi.phi3));


    vmc->inverse_kinematics.J_w_to_v.E.x2_2 = -chassis_physical_config->l4 * cosf(vmc->forward_kinematics.fk_phi.phi0 -
                                                                                  vmc->forward_kinematics.fk_phi.phi2) *
                                              sinf(vmc->forward_kinematics.fk_phi.phi3 -
                                                   vmc->forward_kinematics.fk_phi.phi4)
                                              / (vmc->forward_kinematics.fk_L0.L0 *
                                                 sinf(vmc->forward_kinematics.fk_phi.phi2 -
                                                      vmc->forward_kinematics.fk_phi.phi3));

    vmc->inverse_kinematics.V_fdb.E.last_d_L0_fdb = vmc->inverse_kinematics.V_fdb.E.d_L0_fdb;

    Matrix_multiply(2, 2, vmc->inverse_kinematics.J_w_to_v.array,
                    2, 1, vmc->inverse_kinematics.W_fdb.array,
                    vmc->inverse_kinematics.V_fdb.array);


    vmc->inverse_kinematics.V_fdb.E.dd_L0_fdb =
            (vmc->inverse_kinematics.V_fdb.E.d_L0_fdb - vmc->inverse_kinematics.V_fdb.E.last_d_L0_fdb) /
            (CHASSIS_PERIOD * 0.001f);
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
            vmc->forward_kinematics.fk_L0.L0 *
            sinf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi2)
            / (chassis_physical_config->l1
               * sinf(vmc->forward_kinematics.fk_phi.phi1 - vmc->forward_kinematics.fk_phi.phi2));

    vmc->inverse_kinematics.J_T_to_F.E.x1_2 =
            vmc->forward_kinematics.fk_L0.L0 *
            sinf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi3)
            / (chassis_physical_config->l4
               * sinf(vmc->forward_kinematics.fk_phi.phi4 - vmc->forward_kinematics.fk_phi.phi3));

    vmc->inverse_kinematics.J_T_to_F.E.x2_1 =
            cosf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi2)
            / (chassis_physical_config->l1 *
               sinf(vmc->forward_kinematics.fk_phi.phi2 - vmc->forward_kinematics.fk_phi.phi1));

    vmc->inverse_kinematics.J_T_to_F.E.x2_2 =
            cosf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi3)
            / (chassis_physical_config->l4 *
               sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi4));


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
              + leg->vmc.inverse_kinematics.Fxy_fdb.E.Tp_fdb * sinf(leg->state_variable_feedback.theta) /
                leg->vmc.forward_kinematics.fk_L0.L0;

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

void vmc_calc(float dt) {

    // 更新phi1 phi4
    vmc_phi_update(&chassis.leg_L, &chassis.leg_R);

    // VMC 正运动学解算
    forward_kinematics(&chassis.leg_L, &chassis_physical_config, dt);
    forward_kinematics(&chassis.leg_R, &chassis_physical_config, dt);

//    USART_Vofa_Justfloat_Transmit(chassis.leg_L.vmc.forward_kinematics.fk_phi.phi0, chassis.leg_L.vmc.forward_kinematics.fk_L0.L0);
//    USART_Vofa_Justfloat_Transmit(chassis.leg_R.vmc.forward_kinematics.fk_phi.phi0, chassis.leg_R.vmc.forward_kinematics.fk_L0.L0);

//    USART_Vofa_Justfloat_Transmit(chassis.leg_L.vmc.forward_kinematics.fk_L0.L0_dot, chassis.leg_L.vmc.forward_kinematics.fk_phi.d_phi0);
//    USART_Vofa_Justfloat_Transmit(chassis.leg_R.vmc.forward_kinematics.fk_L0.L0_dot, chassis.leg_R.vmc.forward_kinematics.fk_phi.d_phi0);


    // 腿部协调处理
    leg_coordinate_handle(dt);
}
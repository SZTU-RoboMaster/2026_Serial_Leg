#include <stdbool.h>
#include <math.h>
#include <stdio.h>

#include "robot_def.h"
#include "vmc.h"
#include "user_lib.h"
#include "joint.h"
#include "L0_kalman_filter.h"
#include "phi0_kalman_filter.h"
#include "phi4_kalman_filter.h"
#include "vofa.h"

extern Chassis chassis;
extern ChassisPhysicalConfig chassis_physical_config;

/*               正方向
 *    phi4                      phi4
 *
 *    phi1                      phi1
 */

/* ========================= 1.2 phi1 phi4 phi1',phi4'========================= */

    void vmc_phi_update(Leg *leg_L, Leg *leg_R)
    {

        /* ========================= 1.2.1 初始phi1 phi4 ========================= */

            float LF_joint_pos = (get_joint_motors() + 0)->pos_r;
            float LB_joint_pos = (get_joint_motors() + 1)->pos_r;
            float RF_joint_pos = (get_joint_motors() + 2)->pos_r;
            float RB_joint_pos = (get_joint_motors() + 3)->pos_r;

        /* ========================= 1.2.2 实际phi1 phi4 ========================= */

            /* ========================= 大腿小腿分别向正方向 设对应电机零点 ========================= */

            leg_L->vmc.forward_kinematics.fk_phi.phi1 = LB_joint_pos ; //-0.428
            leg_L->vmc.forward_kinematics.fk_phi.phi4 = LF_joint_pos ; //2.193
            leg_R->vmc.forward_kinematics.fk_phi.phi1 = -(RB_joint_pos - 0.024f); // 0.428 = 0.452+x x=-0.024
            leg_R->vmc.forward_kinematics.fk_phi.phi4 = -(RF_joint_pos -0.047f); //-2.193 = -2.146+y y=-0.047

        /* ========================= 1.2.1 初始phi1‘ phi4’ ========================= */

            float LF_joint_speed = (get_joint_motors() + 0)->angular_vel;
            float LB_joint_speed = (get_joint_motors() + 1)->angular_vel;
            float RF_joint_speed = (get_joint_motors() + 2)->angular_vel;
            float RB_joint_speed = (get_joint_motors() + 3)->angular_vel;

        /* ========================= 1.2.2 实际phi1’ phi4‘ ========================= */

            leg_L->vmc.forward_kinematics.fk_phi.phi4_dot = LF_joint_speed;
            leg_L->vmc.forward_kinematics.fk_phi.phi1_dot = LB_joint_speed;
            leg_R->vmc.forward_kinematics.fk_phi.phi4_dot = -RF_joint_speed;
            leg_R->vmc.forward_kinematics.fk_phi.phi1_dot = -RB_joint_speed;


        /* =========================== 2.4 phi4 phi4'融合 =========================== */

            // PHI4_KF_calc(&PHI4_EstimateKF, &chassis.leg_L, leg_L->vmc.forward_kinematics.fk_phi.phi4_raw, leg_L->vmc.forward_kinematics.fk_phi.phi4_dot_raw);
            // PHI4_KF_calc(&PHI4_EstimateKF, &chassis.leg_R, leg_R->vmc.forward_kinematics.fk_phi.phi4_raw, leg_R->vmc.forward_kinematics.fk_phi.phi4_dot_raw);

    }

/* =========================== 1.3-1.6 正解算 =========================== */

    static void forward_kinematics(Leg *leg, ChassisPhysicalConfig *physical_config)
    {
        /* =========================== 中间变量 =========================== */

        float temp=0,temp_y=0,temp_x=0;

        /* =========================== 1.3  B,D坐标 =========================== */

        leg->vmc.forward_kinematics.fk_point_coordinates.b_x = physical_config->l1 * cosf(leg->vmc.forward_kinematics.fk_phi.phi1);
        leg->vmc.forward_kinematics.fk_point_coordinates.b_y = physical_config->l1 * sinf(leg->vmc.forward_kinematics.fk_phi.phi1);
        leg->vmc.forward_kinematics.fk_point_coordinates.d_x = physical_config->l5 + physical_config->l4 * cosf(leg->vmc.forward_kinematics.fk_phi.phi4);
        leg->vmc.forward_kinematics.fk_point_coordinates.d_y = physical_config->l4 * sinf(leg->vmc.forward_kinematics.fk_phi.phi4);

        /* =========================== 1.4  lBD^2  =========================== */

        float L_BD_sq =
            (leg->vmc.forward_kinematics.fk_point_coordinates.d_x - leg->vmc.forward_kinematics.fk_point_coordinates.b_x)
            * (leg->vmc.forward_kinematics.fk_point_coordinates.d_x - leg->vmc.forward_kinematics.fk_point_coordinates.b_x)
            + (leg->vmc.forward_kinematics.fk_point_coordinates.d_y - leg->vmc.forward_kinematics.fk_point_coordinates.b_y)
            * (leg->vmc.forward_kinematics.fk_point_coordinates.d_y - leg->vmc.forward_kinematics.fk_point_coordinates.b_y);

        /* =========================== 1.5  phi2 =========================== */

        float L_A0 =
            2.0f * physical_config->l2
            * (leg->vmc.forward_kinematics.fk_point_coordinates.d_x
            - leg->vmc.forward_kinematics.fk_point_coordinates.b_x);
        float L_B0 =
            2.0f * physical_config->l2
            * (leg->vmc.forward_kinematics.fk_point_coordinates.d_y
            - leg->vmc.forward_kinematics.fk_point_coordinates.b_y);
        float L_C0 =
            physical_config->l2 * physical_config->l2
            + L_BD_sq
            - physical_config->l3 * physical_config->l3;
        temp = L_A0 * L_A0 + L_B0 * L_B0 - L_C0 * L_C0;
        temp_y = L_B0 - sqrtf(ABS(temp));
        temp_x = L_A0 + L_C0;
        leg->vmc.forward_kinematics.fk_phi.phi2 = 2.0f * atan2f(temp_y, temp_x);

        /* =========================== 1.6  xC,yC =========================== */

        leg->vmc.forward_kinematics.fk_point_coordinates.c_x =
            physical_config->l1 * cosf(leg->vmc.forward_kinematics.fk_phi.phi1)
            + physical_config->l2 * cosf(leg->vmc.forward_kinematics.fk_phi.phi2);
        leg->vmc.forward_kinematics.fk_point_coordinates.c_y =
            physical_config->l1 * sinf(leg->vmc.forward_kinematics.fk_phi.phi1)
            + physical_config->l2 * sinf(leg->vmc.forward_kinematics.fk_phi.phi2);

        /* =========================== 1.7 phi3 =========================== */

        temp_y =
            leg->vmc.forward_kinematics.fk_point_coordinates.c_y
            - leg->vmc.forward_kinematics.fk_point_coordinates.d_y;
        temp_x =
            leg->vmc.forward_kinematics.fk_point_coordinates.c_x
            - leg->vmc.forward_kinematics.fk_point_coordinates.d_x;
        leg->vmc.forward_kinematics.fk_phi.phi3 = atan2f(temp_y, temp_x);

        /* =========================== 1.8  L0 =========================== */

        temp =
            (leg->vmc.forward_kinematics.fk_point_coordinates.c_x - physical_config->l5 * 0.5f)
            * (leg->vmc.forward_kinematics.fk_point_coordinates.c_x - physical_config->l5 * 0.5f)
            + leg->vmc.forward_kinematics.fk_point_coordinates.c_y
            * leg->vmc.forward_kinematics.fk_point_coordinates.c_y;
        leg->vmc.forward_kinematics.fk_L0.L0 = sqrtf(ABS(temp));

        /* =========================== 1.8  phi0 =========================== */

        temp_y = leg->vmc.forward_kinematics.fk_point_coordinates.c_y;
        temp_x = leg->vmc.forward_kinematics.fk_point_coordinates.c_x - physical_config->l5 * 0.5f;
        leg->vmc.forward_kinematics.fk_phi.phi0 = atan2f(temp_y, temp_x);


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


        // /* =========================== L0 L0'融合 =========================== */
        //
        // L0_KF_calc(&L0_EstimateKF, leg, leg->vmc.forward_kinematics.fk_L0.L0_raw, leg->vmc.forward_kinematics.fk_L0.L0_dot_raw);
        //
        // /**  6、腿长L0_ddot差分求解 **/
        // leg->vmc.forward_kinematics.fk_L0.L0_ddot =
        //         (leg->vmc.forward_kinematics.fk_L0.L0_dot - leg->vmc.forward_kinematics.fk_L0.L0_dot_last)
        //         / (CHASSIS_PERIOD * 0.001f);
        // leg->vmc.forward_kinematics.fk_L0.L0_ddot_raw = update_low_pass_filter(&leg->L0_ddot,leg->vmc.forward_kinematics.fk_L0.L0_ddot);
        //
        // /* =========================== 低通滤波 =========================== */
        //
        // leg->vmc.forward_kinematics.fk_L0.L0_ddot = update_low_pass_filter(&leg->L0_ddot,leg->vmc.forward_kinematics.fk_L0.L0_ddot_raw);


        /** 8、精确求解phi0_dot **/
        // 求phi0_dot
        temp_y = (leg->vmc.forward_kinematics.fk_point_coordinates.c_x - (physical_config->l5 / 2.0f)) *
                 (Yb_dot + A1 * cosf(leg->vmc.forward_kinematics.fk_phi.phi2)) -
                 leg->vmc.forward_kinematics.fk_point_coordinates.c_y *
                 (Xb_dot - A1 * sinf(leg->vmc.forward_kinematics.fk_phi.phi2));
        temp_x = leg->vmc.forward_kinematics.fk_L0.L0 * leg->vmc.forward_kinematics.fk_L0.L0;

        leg->vmc.forward_kinematics.fk_phi.last_d_phi0 = leg->vmc.forward_kinematics.fk_phi.d_phi0;
        leg->vmc.forward_kinematics.fk_phi.d_phi0 = temp_y / temp_x;

        // /* =========================== phi0 phi0'融合 =========================== */

        // PHI0_KF_calc(&PHI0_EstimateKF, &chassis.leg_L, chassis.leg_L.vmc.forward_kinematics.fk_phi.phi0_raw, chassis.leg_L.vmc.forward_kinematics.fk_phi.d_phi0_raw);
        // PHI0_KF_calc(&PHI0_EstimateKF, leg, leg->vmc.forward_kinematics.fk_phi.phi0_raw, leg->vmc.forward_kinematics.fk_phi.d_phi0_raw);

        /**  9、phi0_ddot 差分求解 **/
        leg->vmc.forward_kinematics.fk_phi.dd_phi0 =
                (leg->vmc.forward_kinematics.fk_phi.d_phi0 - leg->vmc.forward_kinematics.fk_phi.last_d_phi0)
                / (CHASSIS_PERIOD * 0.001f);

        // /* =========================== 低通滤波 =========================== */

        // leg->vmc.forward_kinematics.fk_phi.dd_phi0 = update_low_pass_filter(&leg->phi0_ddot,leg->vmc.forward_kinematics.fk_phi.dd_phi0_raw);
    }

/* =========================== 腿部协调处理 =========================== */

    static void leg_coordinate_handle(void)
    {

        /* =========================== 储存φ_0差 =========================== */

            chassis.last_phi0_error = chassis.phi0_error;

        /* =========================== 左右腿φ_0差 =========================== */

            chassis.phi0_error =
                chassis.leg_L.vmc.forward_kinematics.fk_phi.phi0
                - chassis.leg_R.vmc.forward_kinematics.fk_phi.phi0;

        /* =========================== φ_0差' =========================== */

            chassis.d_phi0_error =
                (chassis.phi0_error - chassis.last_phi0_error)
                / (CHASSIS_PERIOD * 0.001f);
    }

/* =========================== 2. 正动力学变换 =========================== */

    void vmc_forward_dynamics(VMC *vmc, const ChassisPhysicalConfig *physical_config)
    {
         if (vmc == NULL)
         {
             return;
         }

        /* =========================== 矩阵 =========================== */

            vmc->forward_kinematics.J_F_to_T.E.x1_1 =
                    physical_config->l1
                    * cosf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi3)
                    * sinf(vmc->forward_kinematics.fk_phi.phi1 - vmc->forward_kinematics.fk_phi.phi2)
                    / (vmc->forward_kinematics.fk_L0.L0
                    * sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi2));

            vmc->forward_kinematics.J_F_to_T.E.x1_2 =
                    physical_config->l1
                    * sinf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi3)
                    * sinf(vmc->forward_kinematics.fk_phi.phi1 - vmc->forward_kinematics.fk_phi.phi2)
                    / sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi2);

            vmc->forward_kinematics.J_F_to_T.E.x2_1 =
                    physical_config->l4
                    * cosf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi2)
                    * sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi4)
                    / (vmc->forward_kinematics.fk_L0.L0
                    * sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi2));

            vmc->forward_kinematics.J_F_to_T.E.x2_2 =
                    physical_config->l4
                    * sinf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi2)
                    * sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi4)
                    / sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi2);


        /* =========================== T1,T4 =========================== */

            Matrix_multiply(2, 2, vmc->forward_kinematics.J_F_to_T.array,
                            2, 1, vmc->forward_kinematics.Fxy_set_point.array,
                            vmc->forward_kinematics.T1_T4_set_point.array);

        // USART_Vofa_Justfloat_Transmit(
        //         chassis.leg_L.vmc.forward_kinematics.J_F_to_T.E.x1_1,chassis.leg_L.vmc.forward_kinematics.J_F_to_T.E.x1_2
        //      ,chassis.leg_L.vmc.forward_kinematics.J_F_to_T.E.x2_1,chassis.leg_L.vmc.forward_kinematics.J_F_to_T.E.x2_2
        //
        //      ,chassis.leg_R.vmc.forward_kinematics.J_F_to_T.E.x1_1,chassis.leg_R.vmc.forward_kinematics.J_F_to_T.E.x1_2
        //      ,chassis.leg_R.vmc.forward_kinematics.J_F_to_T.E.x2_1,chassis.leg_R.vmc.forward_kinematics.J_F_to_T.E.x2_2
        //
        //     ,chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.array[0][0],chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.array[0][1]
        //     ,chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point,chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point
        //
        //     ,chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.array[0][0],chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.array[0][1]
        //     ,chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point,chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point
        //
        //     ,0,0,0,0
        //     ,0,0,0,0
        //     ,0,0,0,0
        //     ,0,0,0,0
        //     ,0,0,0,0
        //     ,0,0,0);

    }

// 逆解算腿长变化速度、摆角变化速度θ'

static void vmc_inverse_kinematics(VMC *vmc, float w1, float w4, ChassisPhysicalConfig *chassis_physical_config)
{
    if (vmc == NULL)
    {
        return;
    }

    vmc->inverse_kinematics.W_fdb.E.w1_fdb = w1;
    vmc->inverse_kinematics.W_fdb.E.w4_fdb = w4;

    vmc->inverse_kinematics.J_w_to_v.E.x1_1 =
        -chassis_physical_config->l1
        * sinf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi3)
        * sinf(vmc->forward_kinematics.fk_phi.phi1 - vmc->forward_kinematics.fk_phi.phi2)
        / sinf(vmc->forward_kinematics.fk_phi.phi2 - vmc->forward_kinematics.fk_phi.phi3);

    vmc->inverse_kinematics.J_w_to_v.E.x1_2 =
        -chassis_physical_config->l4
        * sinf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi2)
        * sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi4)
        / sinf(vmc->forward_kinematics.fk_phi.phi2 - vmc->forward_kinematics.fk_phi.phi3);

    vmc->inverse_kinematics.J_w_to_v.E.x2_1 =
        -chassis_physical_config->l1
        * cosf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi3)
        * sinf(vmc->forward_kinematics.fk_phi.phi1 - vmc->forward_kinematics.fk_phi.phi2)
        / (vmc->forward_kinematics.fk_L0.L0
        * sinf(vmc->forward_kinematics.fk_phi.phi2 - vmc->forward_kinematics.fk_phi.phi3));

    vmc->inverse_kinematics.J_w_to_v.E.x2_2 =
        -chassis_physical_config->l4
        * cosf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi2)
        * sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi4)
        / (vmc->forward_kinematics.fk_L0.L0
        * sinf(vmc->forward_kinematics.fk_phi.phi2 - vmc->forward_kinematics.fk_phi.phi3));

    vmc->inverse_kinematics.V_fdb.E.last_d_L0_fdb = vmc->inverse_kinematics.V_fdb.E.d_L0_fdb;

    Matrix_multiply(2, 2, vmc->inverse_kinematics.J_w_to_v.array,
                    2, 1, vmc->inverse_kinematics.W_fdb.array,
                    vmc->inverse_kinematics.V_fdb.array);

    vmc->inverse_kinematics.V_fdb.E.dd_L0_fdb =
            (vmc->inverse_kinematics.V_fdb.E.d_L0_fdb - vmc->inverse_kinematics.V_fdb.E.last_d_L0_fdb)
            / (CHASSIS_PERIOD * 0.001f);

}

// 逆解算出虚拟力矩和沿腿方向支持力
static void vmc_inverse_dynamics(VMC *vmc,float T1,float T4,ChassisPhysicalConfig *chassis_physical_config)
{
    if (vmc == NULL)
    {
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

// 计算竖直方向支持力 用于离地检测
static void fn_cal(Leg *leg, float body_az, ChassisPhysicalConfig *chassis_physical_config)
{
    if (leg == NULL)
    {
        return;
    }

    // 用逆解算的数据计算
    float P = leg->vmc.inverse_kinematics.Fxy_fdb.E.Fy_fdb * cosf(leg->state_variable_feedback.theta)     /* kss：没准有个负号 */
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

/* =========================== 1. 正解算 =========================== */

    void vmc_calc(void)
    {

        /* ========================= 1.2 phi1 phi4 phi1',phi4'========================= */

            vmc_phi_update(&chassis.leg_L, &chassis.leg_R);

        /* =========================== 1.3-1.6 正解算 =========================== */

            forward_kinematics(&chassis.leg_L, &chassis_physical_config);
            forward_kinematics(&chassis.leg_R, &chassis_physical_config);

        /* =========================== 腿部协调处理 =========================== */

            leg_coordinate_handle();
    }
#include <math.h>
#include "chassis_task.h"
#include "robot_def.h"
#include "user_lib.h"
#include "joint.h"
#include "DJI_motor.h"
#include "wheel.h"
#include "remote.h"
#include "vmc.h"
#include "error.h"
#include "speed_kalman_filter.h"
#include "theta_kalman_filter.h"
#include "lqr.h"
#include "bsp_dwt.h"
#include "vofa.h"
#include "dm_imu.h"
#include "ins_task.h"
#include "L0_kalman_filter.h"
#include "phi0_kalman_filter.h"
#include "phi4_kalman_filter.h"

/*******************************************************************************************************************

    函数跳转情况：
    |
    任务
       |
       ——chassis_init
                    |
                    ——(joint_init)
                    ——(wheel_init)
                    ——chassis_pid_init
                    ——(INS_Init)
                    ——(Theta_EstimateKF_Init)
                    ——(low_pass_filter_init)
                    ——(Speed_EstimateKF_Init)
                    ——(low_pass_filter_init)
       |
       ——chassis_task
                    |
                    ——chassis_remote_cmd
                                        |
                                        ——set_chassis_mode
                                        ——set_chassis_ctrl_info
                    ——chassis_observer_update
                                            |
                                            ——get_IMU_info
                                                            |
                                                            ——(Body_Accel_To_Earth)
                                            ——chassis_variable_update
                                                                    |
                                                                    ——(get_wheel_motors)
                                                                    ——(Speed_KF_calc)
                                                                    ——(update_low_pass_filter)
                                                                    ——(Theta_KF_calc)
                                                                    ——(update_low_pass_filter)
                                            ——(vmc_calc)
                                                        |
                                                        ——(vmc_phi_update)
                                                        ——(forward_kinematics)
                                                        ——(leg_coordinate_handle)
                    ——chassis_disable_task
                    ——chassis_init_task
                                        |
                                        ——(joint_enable)
                    ——chassis_enable_task
                                        |
                                        ——wheel_calc
                                                    |
                                                    ——(chassis_K_matrix_fitting)
                                                    ——(pid_calc)
                                                    ——(VAL_LIMIT)
                                        ——joint_calc
                                                    |
                                                    ——(pid_calc)
                                                    ——(chassis_K_matrix_fitting)
                                                    ——(vmc_forward_dynamics)
                                                                        |
                                                                        ——(Matrix_multiply)
                                                    ——(VAL_LIMIT)
                                        ——chassis_selfhelp
                    ——MIT_send_torque_task
    ——中断
         |
         ——(SBUS_TO_RC)
         ——(imu_data_unpack)

    相关文件：
        dm_imu.c dm_imu.h 达妙IMU ins_task.c 姿态解算
        low_pass_filter.c 低通滤波 kalman filter.h 卡尔曼 pid.c PID
        speed_kalman_filter.c/h theta_kalman_filter.c/h 卡尔曼应用
        robot_def.c robot_def.h 相关定义
        lqr.c LQR vmc.c VMC 运动建模
        wheel.c joint.c/h DJI_motor.h 电机
        remote.c remote.h user_lib.h遥控
        bsp_usart.c 中断 freertos.c 任务

    没用到但后续可以尝试用的：
    controller.c/h和pid.c/h中的进阶pid
    QuaternionEKF四元数 用于姿态解算（AHRS)
    user_lib

*******************************************************************************************************************/


/*******************************************************************************
 *                                  初始化                                       *
 *******************************************************************************/

/* =========================== 底盘pid初始化 ============================= */

    static void chassis_pid_init(void)
    {

        /* =========================== 轮毂 ============================= */

            /* =========================== 转向-PID ============================= */

                /* =========================== 转向-位置环-PID ============================= */

                    pid_init(&chassis.chassis_turn_pos_pid,
                             CHASSIS_TURN_POS_PID_OUT_LIMIT,
                             CHASSIS_TURN_POS_PID_IOUT_LIMIT,
                             CHASSIS_TURN_POS_PID_P,
                             CHASSIS_TURN_POS_PID_I,
                             CHASSIS_TURN_POS_PID_D);

                /* =========================== 转向-速度环-PID ============================= */

                    pid_init(&chassis.chassis_turn_speed_pid,
                             CHASSIS_TURN_SPEED_PID_OUT_LIMIT,
                             CHASSIS_TURN_SPEED_PID_IOUT_LIMIT,
                             CHASSIS_TURN_SPEED_PID_P,
                             CHASSIS_TURN_SPEED_PID_I,
                             CHASSIS_TURN_SPEED_PID_D);

        /* =========================== 关节 ============================= */

            /* =========================== 防劈叉-PID ============================= */

                pid_init(&chassis.chassis_leg_coordination_pid,
                         CHASSIS_LEG_COORDINATION_PID_OUT_LIMIT,
                         CHASSIS_LEG_COORDINATION_PID_IOUT_LIMIT,
                         CHASSIS_LEG_COORDINATION_PID_P,
                         CHASSIS_LEG_COORDINATION_PID_I,
                         CHASSIS_LEG_COORDINATION_PID_D);

            /* =========================== 腿长-PID ============================= */

                /* iss：后面最好分段算 */

                /* =========================== 腿长-位置环-PID ============================= */

                    pid_init(&chassis.leg_L.leg_pos_pid,
                             CHASSIS_LEG_L0_POS_PID_OUT_LIMIT,
                             CHASSIS_LEG_L0_POS_PID_IOUT_LIMIT,
                             CHASSIS_LEG_L0_POS_PID_P,
                             CHASSIS_LEG_L0_POS_PID_I,
                             CHASSIS_LEG_L0_POS_PID_D);

                    pid_init(&chassis.leg_R.leg_pos_pid,
                             CHASSIS_LEG_L0_POS_PID_OUT_LIMIT,
                             CHASSIS_LEG_L0_POS_PID_IOUT_LIMIT,
                             CHASSIS_LEG_L0_POS_PID_P,
                             CHASSIS_LEG_L0_POS_PID_I,
                             CHASSIS_LEG_L0_POS_PID_D);

                /* =========================== 腿长-速度环-PID ============================= */

                    pid_init(&chassis.leg_L.leg_speed_pid,
                             CHASSIS_LEG_L0_SPEED_PID_OUT_LIMIT,
                             CHASSIS_LEG_L0_SPEED_PID_IOUT_LIMIT,
                             CHASSIS_LEG_L0_SPEED_PID_P,
                             CHASSIS_LEG_L0_SPEED_PID_I,
                             CHASSIS_LEG_L0_SPEED_PID_D);

                    pid_init(&chassis.leg_R.leg_speed_pid,
                             CHASSIS_LEG_L0_SPEED_PID_OUT_LIMIT,
                             CHASSIS_LEG_L0_SPEED_PID_IOUT_LIMIT,
                             CHASSIS_LEG_L0_SPEED_PID_P,
                             CHASSIS_LEG_L0_SPEED_PID_I,
                             CHASSIS_LEG_L0_SPEED_PID_D);

            /* =========================== Roll补偿-PID ============================= */

                pid_init(&chassis.chassis_roll_pid,
                         CHASSIS_ROLL_PID_OUT_LIMIT,
                         CHASSIS_ROLL_PID_IOUT_LIMIT,
                         CHASSIS_ROLL_PID_P,
                         CHASSIS_ROLL_PID_I,
                         CHASSIS_ROLL_PID_D);

        /* =========================== 自救θ-PID =========================== */

            pid_init(&chassis.leg_L.leg_theta_reset_pid,
                    CHASSIS_THETA_PID_OUT_LIMIT,
                    CHASSIS_THETA_PID_IOUT_LIMIT,
                             CHASSIS_THETA_PID_P,
                             CHASSIS_THETA_PID_I,
                             CHASSIS_THETA_PID_D);

            pid_init(&chassis.leg_R.leg_theta_reset_pid,
                    CHASSIS_THETA_PID_OUT_LIMIT,
                    CHASSIS_THETA_PID_IOUT_LIMIT,
                             CHASSIS_THETA_PID_P,
                             CHASSIS_THETA_PID_I,
                             CHASSIS_THETA_PID_D);

    }

/*******************************************************************************
 *                                 遥控                                      *
 *******************************************************************************/

/* iss：后面移到remote.c 通过板间通信接收 */

/* =========================== 底盘根据遥控器设置模式 =========================== */

    static void set_chassis_mode(void)
    {

        /* =========================== 右拨杆在下面 失能 =========================== */

            if (switch_is_down(remote_ctrl.rc.s[RC_s_R]))
            {
                chassis.chassis_last_ctrl_mode = chassis.chassis_ctrl_mode;
                chassis.chassis_ctrl_mode = CHASSIS_DISABLE;
            }

        /* =========================== 右拨杆刚刚拨到中间 初始化 =========================== */

            else if (switch_is_mid(remote_ctrl.rc.s[RC_s_R]) && (chassis.init_flag == false))
            {
                chassis.chassis_last_ctrl_mode = chassis.chassis_ctrl_mode;
                chassis.chassis_ctrl_mode = CHASSIS_INIT;
            }

        /* =========================== 右拨杆在中间 使能 =========================== */

            else if (switch_is_mid(remote_ctrl.rc.s[RC_s_R]) && (chassis.init_flag == true))
            {
                chassis.chassis_last_ctrl_mode = chassis.chassis_ctrl_mode;
                chassis.chassis_ctrl_mode = CHASSIS_ENABLE;
            }

        /* =========================== 右拨杆在上面 小陀螺 =========================== */

            else if (switch_is_up(remote_ctrl.rc.s[RC_s_R]))
            {
                chassis.chassis_last_ctrl_mode = chassis.chassis_ctrl_mode;
                chassis.chassis_ctrl_mode = CHASSIS_SPIN;
            }

    }

/* =========================== 底盘接收遥控器信息 =========================== */

    static void set_chassis_ctrl_info(void)
    {

        /* =========================== 期望速度 =========================== */

            float vel_temp= - (float) (remote_ctrl.rc.ch[CHASSIS_VX_CHANNEL]) * RC_TO_VX;

            /* 防猛冲 */

                slope_following(&vel_temp,&chassis.chassis_ctrl_info.v_m_per_s,0.02f);

            /* iss：测试 */

                // chassis.chassis_ctrl_info.v_m_per_s = 0;

        /* =========================== 转向 期望yaw 在上次下电的yaw的基础上 =========================== */

            chassis.chassis_ctrl_info.yaw_rad += (float) (remote_ctrl.rc.ch[CHASSIS_YAW_CHANNEL]) * RC_TO_YAW_INCREMENT / (2 * PI);

        /* =========================== 期望腿长 =========================== */

            // /* iss：待测试 */
            //
            //     if(remote_ctrl.rc.ch[CHASSIS_LEG_CHANNEL] == -660)
            //     {
            //         chassis.chassis_ctrl_info.target_length = 0.12f;
            //     }
            //     else if(remote_ctrl.rc.ch[CHASSIS_LEG_CHANNEL] == 660)
            //     {
            //         chassis.chassis_ctrl_info.target_length = 0.32f;
            //     }
            //     else
            //     {
            //         chassis.chassis_ctrl_info.target_length = 0.18f;
            //     }

            /* iss：测试 */

                chassis.chassis_ctrl_info.target_length = 0.185f;

            /* iss：后续可以根据pitch自适应 */


    }

/* =========================== 获取底盘遥控器信息 =========================== */

    void chassis_remote_cmd(void)
    {

        /* =========================== 底盘根据遥控器设置模式 =========================== */

        set_chassis_mode();

        /* =========================== 底盘接收遥控器信息 =========================== */

        set_chassis_ctrl_info();
    }

/***************************************************************************
 *                                观测                                     *
 ***************************************************************************/

/* =========================== 获取底盘传感器数据 =========================== */

    /* iss：后续可以用EKF */

    static void get_IMU_info(void)
    {

        /* =========================== Yaw =========================== */

            chassis.imu_reference.yaw_rad = - DM_IMU.yaw * DEGREE_TO_RAD;

            // chassis.imu_reference.yaw_rad = QEKF_INS.Yaw * DEGREE_TO_RAD;

            /* =========================== 圈数 =========================== */

                if (chassis.imu_reference.yaw_rad - chassis.imu_reference.yaw_last_rad > PI)
                {
                    chassis.imu_reference.yaw_round_count--;
                }
                else if (chassis.imu_reference.yaw_rad - chassis.imu_reference.yaw_last_rad < -PI)
                {
                    chassis.imu_reference.yaw_round_count++;
                }

            /* =========================== 实际yaw =========================== */

                chassis.imu_reference.yaw_total_rad =
                    2 * PI * chassis.imu_reference.yaw_round_count
                    + chassis.imu_reference.yaw_rad;

                // chassis.imu_reference.yaw_total_rad =
                //      QEKF_INS.YawTotalAngle * DEGREE_TO_RAD;

            /* =========================== 储存 供下次计算圈数 =========================== */

                chassis.imu_reference.yaw_last_rad = chassis.imu_reference.yaw_rad;

        /* =========================== Pitch =========================== */

            chassis.imu_reference.pitch_rad = - DM_IMU.roll * DEGREE_TO_RAD;

            // chassis.imu_reference.pitch_rad = QEKF_INS.Roll * DEGREE_TO_RAD;

        /* =========================== Roll =========================== */

            chassis.imu_reference.roll_rad = - DM_IMU.pitch * DEGREE_TO_RAD;

            // chassis.imu_reference.roll_rad = QEKF_INS.Pitch * DEGREE_TO_RAD;

        /* =========================== 更新各轴角速度(rad/s) =========================== */

            chassis.imu_reference.yaw_gyro = - DM_IMU.gyro[2];
            chassis.imu_reference.pitch_gyro = - DM_IMU.gyro[0];
            chassis.imu_reference.roll_gyro = - DM_IMU.gyro[1];

            // chassis.imu_reference.pitch_gyro = QEKF_INS.Gyro[2];
            // chassis.imu_reference.yaw_gyro = QEKF_INS.Gyro[0];
            // chassis.imu_reference.roll_gyro = QEKF_INS.Gyro[1];

        /* =========================== 更新各轴加速度 =========================== */

            /* =========================== 更新各轴加速度 =========================== */

                chassis.imu_reference.ax = - DM_IMU.accel[1];
                chassis.imu_reference.ay = - DM_IMU.accel[0];
                chassis.imu_reference.az = - DM_IMU.accel[2];

                // chassis.imu_reference.ax = QEKF_INS.Accel[1];
                // chassis.imu_reference.ay = QEKF_INS.Accel[0];
                // chassis.imu_reference.az = QEKF_INS.Accel[2];


            /* =========================== 惯导解算：将机体坐标系的加速度转换为世界坐标系(地面） =========================== */

                Body_Accel_To_Earth();

    }

/* =========================== 更新底盘状态变量X =========================== */

    static void chassis_variable_update(void)
    {

        /* =========================== φ φ' =========================== */

            /* =========================== φ =========================== */

                chassis.leg_L.state_variable_feedback.phi = chassis.imu_reference.pitch_rad;
                chassis.leg_R.state_variable_feedback.phi = chassis.imu_reference.pitch_rad;

            /* =========================== φ'=========================== */

                chassis.leg_L.state_variable_feedback.phi_dot = chassis.imu_reference.pitch_gyro;
                chassis.leg_R.state_variable_feedback.phi_dot = chassis.imu_reference.pitch_gyro;

        /* =========================== x x' x'' =========================== */

            /* =========================== x' =========================== */

                /* =========================== 上次的x' =========================== */

                    chassis.leg_L.state_variable_feedback.x_dot_last = chassis.leg_L.state_variable_feedback.x_dot;
                    chassis.leg_R.state_variable_feedback.x_dot_last = chassis.leg_R.state_variable_feedback.x_dot;

                /* =========================== 坐标系转换 c系：轮毂电机定子 b系：机体 e系：大地 =========================== */

                    /* =========================== 1. 角速度 =========================== */

                        /* =========================== 1.1 电机编码器反馈角速度 （c系 转子相对定子）  =========================== */

                            chassis.leg_L.w_ecd = get_wheel_motors()->speed_rpm / RATIO * RPM_TO_RAD_PER_S;
                            chassis.leg_R.w_ecd = (get_wheel_motors() + 1)->speed_rpm / RATIO * RPM_TO_RAD_PER_S ;

                        /* =========================== 1.1 根据机械结构加负号 =========================== */

                            chassis.leg_R.w_ecd *= -1;

                        /* =========================== 1.1 定子角速度 （c系相对于b系） =========================== */

                            chassis.leg_L.phi_bc_dot = chassis.leg_L.vmc.forward_kinematics.fk_phi.d_phi0;
                            chassis.leg_R.phi_bc_dot = chassis.leg_R.vmc.forward_kinematics.fk_phi.d_phi0;

                        /* =========================== 1.1 机体角速度 （b系相对于e系） =========================== */

                            chassis.leg_L.w_eb = chassis.leg_L.state_variable_feedback.phi_dot;
                            chassis.leg_R.w_eb = chassis.leg_R.state_variable_feedback.phi_dot;

                        /* =========================== 1.2 轮毂（c系）相对大地（e系）的角速度 =========================== */

                            chassis.leg_L.w = chassis.leg_L.w_ecd - chassis.leg_L.phi_bc_dot - chassis.leg_L.w_eb;
                            chassis.leg_R.w = chassis.leg_R.w_ecd - chassis.leg_R.phi_bc_dot - chassis.leg_R.w_eb;

                    /* =========================== 2. 速度 =========================== */

                        /* =========================== 2.1 轮毂速度 =========================== */

                            chassis.leg_L.v = chassis.leg_L.w * chassis_physical_config.wheel_radius;
                            chassis.leg_R.v = chassis.leg_R.w * chassis_physical_config.wheel_radius;

                        /* =========================== 2.2 两个机体速度 =========================== */

                            chassis.leg_L.v_b = chassis.leg_L.v
                                + chassis.leg_L.vmc.forward_kinematics.fk_L0.L0
                                * chassis.leg_L.state_variable_feedback.theta_dot
                                * cosf(chassis.leg_L.state_variable_feedback.theta)
                                + chassis.leg_L.vmc.forward_kinematics.fk_L0.L0_dot
                                * sinf(chassis.leg_L.state_variable_feedback.theta);

                            chassis.leg_R.v_b = chassis.leg_R.v
                                + chassis.leg_R.vmc.forward_kinematics.fk_L0.L0
                                * chassis.leg_R.state_variable_feedback.theta_dot
                                * cosf(chassis.leg_R.state_variable_feedback.theta)
                                + chassis.leg_R.vmc.forward_kinematics.fk_L0.L0_dot
                                * sinf(chassis.leg_R.state_variable_feedback.theta);

                        /* =========================== 2.3 取平均计算出机体速度 =========================== */

                            chassis.leg_L.state_variable_feedback.x_dot_raw = (chassis.leg_L.v_b + chassis.leg_R.v_b) / 2;
                            chassis.leg_R.state_variable_feedback.x_dot_raw = (chassis.leg_L.v_b + chassis.leg_R.v_b) / 2;

                /* =========================== 2.4 x' x''融合 估计x' =========================== */

                    chassis.leg_L.state_variable_feedback.x_dot = chassis.leg_L.state_variable_feedback.x_dot_raw;
                    chassis.leg_R.state_variable_feedback.x_dot = chassis.leg_R.state_variable_feedback.x_dot_raw;

                    /* iss：先不融合 */

                        // Speed_KF_calc(&Speed_EstimateKF, &chassis.leg_L, chassis.leg_L.state_variable_feedback.x_dot_raw, chassis.imu_reference.robot_ax);
                        // Speed_KF_calc(&Speed_EstimateKF, &chassis.leg_R, chassis.leg_R.state_variable_feedback.x_dot_raw, chassis.imu_reference.robot_ax);

            /* =========================== x =========================== */

                /* =========================== 如果在遥控 不设置x =========================== */

                if (chassis.chassis_ctrl_info.v_m_per_s != 0.0f)
                {
                    chassis.leg_L.state_variable_feedback.x = 0.0f;
                    chassis.leg_R.state_variable_feedback.x = 0.0f;
                }

                /* =========================== 否则 求积分得到x =========================== */

                else
                {
                    chassis.leg_L.state_variable_feedback.x +=
                            (CHASSIS_PERIOD * 0.001f) * chassis.leg_L.state_variable_feedback.x_dot;
                    chassis.leg_R.state_variable_feedback.x +=
                            (CHASSIS_PERIOD * 0.001f) * chassis.leg_R.state_variable_feedback.x_dot;
                }

            /* =========================== x'' =========================== */

                /* =========================== 求导 =========================== */

                    float L_x_ddot_raw =
                            (chassis.leg_L.state_variable_feedback.x_dot - chassis.leg_L.state_variable_feedback.x_dot_last)
                            /(CHASSIS_PERIOD * 0.001f);

                    float R_x_ddot_raw =
                            (chassis.leg_R.state_variable_feedback.x_dot - chassis.leg_R.state_variable_feedback.x_dot_last)
                            /(CHASSIS_PERIOD * 0.001f);

                /* =========================== 低通滤波 =========================== */

                    chassis.leg_L.state_variable_feedback.x_ddot = update_low_pass_filter(&chassis.leg_L.x_ddot, L_x_ddot_raw);
                    chassis.leg_R.state_variable_feedback.x_ddot = update_low_pass_filter(&chassis.leg_R.x_ddot, R_x_ddot_raw);

        /* =========================== θ θ' θ'' =========================== */

            /* =========================== θ =========================== */

                /* =========================== α =========================== */

                    chassis.leg_L.state_variable_feedback.alpha =  PI / 2 - chassis.leg_L.vmc.forward_kinematics.fk_phi.phi0;
                    chassis.leg_R.state_variable_feedback.alpha =  PI / 2 - chassis.leg_R.vmc.forward_kinematics.fk_phi.phi0;

                /* =========================== θ =========================== */

                    float L_theta_raw = - chassis.leg_L.state_variable_feedback.alpha - chassis.leg_L.state_variable_feedback.phi;
                    float R_theta_raw = - chassis.leg_R.state_variable_feedback.alpha - chassis.leg_R.state_variable_feedback.phi;

            /* =========================== θ' =========================== */

                /* =========================== 上次的θ' =========================== */

                    chassis.leg_L.state_variable_feedback.theta_dot_last = chassis.leg_L.state_variable_feedback.theta_dot;
                    chassis.leg_R.state_variable_feedback.theta_dot_last = chassis.leg_R.state_variable_feedback.theta_dot;

                /* =========================== θ' =========================== */

                    float L_theta_dot_raw =  chassis.leg_L.vmc.forward_kinematics.fk_phi.d_phi0
                                            - chassis.leg_L.state_variable_feedback.phi_dot;
                    float R_theta_dot_raw =  chassis.leg_R.vmc.forward_kinematics.fk_phi.d_phi0
                                            - chassis.leg_R.state_variable_feedback.phi_dot;

        /* =========================== 2.4 θ θ' 融合 估计 θ θ' =========================== */

                chassis.leg_L.state_variable_feedback.theta = L_theta_raw;
                chassis.leg_L.state_variable_feedback.theta_dot = L_theta_dot_raw;

                chassis.leg_R.state_variable_feedback.theta = R_theta_raw;
                chassis.leg_R.state_variable_feedback.theta_dot = R_theta_dot_raw;

                /* iss：先不融合 */

                    // theta_KF_calc(&Theta_EstimateKF_LegL,&chassis.leg_L, L_theta_raw, L_theta_dot_raw);
                    // theta_KF_calc(&Theta_EstimateKF_LegR,&chassis.leg_R, R_theta_raw, R_theta_dot_raw);

            /* =========================== θ'' =========================== */

                /* =========================== 求导 =========================== */

                    float L_theta_ddot_raw =
                        (chassis.leg_L.state_variable_feedback.theta_dot - chassis.leg_L.state_variable_feedback.theta_dot_last)
                        /(CHASSIS_PERIOD * 0.001f);
                    float R_theta_ddot_raw = chassis.leg_R.state_variable_feedback.theta_ddot =
                        (chassis.leg_R.state_variable_feedback.theta_dot - chassis.leg_R.state_variable_feedback.theta_dot_last)
                        /(CHASSIS_PERIOD * 0.001f);

                /* =========================== 低通滤波 =========================== */

                    chassis.leg_L.state_variable_feedback.theta_ddot = update_low_pass_filter(&chassis.leg_L.theta_ddot, L_theta_ddot_raw);
                    chassis.leg_R.state_variable_feedback.theta_ddot = update_low_pass_filter(&chassis.leg_R.theta_ddot, R_theta_ddot_raw);

    }

/* =========================== 底盘观测器更新 ====================== */

    static void chassis_observer_update(void)
    {

        /* =========================== 获取传感器数据 ====================== */

            get_IMU_info();

        /* =========================== 更新底盘状态变量X =========================== */

            chassis_variable_update();

        /* =========================== 1. 正解算 =========================== */

            vmc_calc();

    }

/*****************************************************************************
 *                                 解算                                      *
 ****************************************************************************/

/* ============================= 4. 计算力矩 轮毂 ============================= */

    static void wheel_calc(void)
    {

        chassis.leg_L.wheel_torque = 0;
        chassis.leg_R.wheel_torque = 0;

        /* ============================= 3.1 不同腿长下进行多项式拟合 ============================= */

            chassis_K_matrix_fitting(chassis.leg_L.vmc.forward_kinematics.fk_L0.L0, wheel_K_L, wheel_fitting_factor);
            chassis_K_matrix_fitting(chassis.leg_R.vmc.forward_kinematics.fk_L0.L0, wheel_K_R, wheel_fitting_factor);

        /* ============================= 3.2 加入目标输入 ============================= */

            chassis.leg_L.wheel_torque_lqr =
                - wheel_K_L[0] * (chassis.leg_L.state_variable_feedback.theta - 0.0f)
                - wheel_K_L[1] * (chassis.leg_L.state_variable_feedback.theta_dot - 0.0f)
                - wheel_K_L[2] * (chassis.leg_L.state_variable_feedback.x - 0.0f)
                - wheel_K_L[3] * (chassis.leg_L.state_variable_feedback.x_dot - chassis.chassis_ctrl_info.v_m_per_s)
                - wheel_K_L[4] * (chassis.leg_L.state_variable_feedback.phi - 0.0f)
                - wheel_K_L[5] * (chassis.leg_L.state_variable_feedback.phi_dot - 0.0f)
                ;

            chassis.leg_R.wheel_torque_lqr =
                - wheel_K_R[0] * (chassis.leg_R.state_variable_feedback.theta - 0.0f)
                - wheel_K_R[1] * (chassis.leg_R.state_variable_feedback.theta_dot - 0.0f)
                - wheel_K_R[2] * (chassis.leg_R.state_variable_feedback.x - 0.0f)
                - wheel_K_R[3] * (chassis.leg_R.state_variable_feedback.x_dot - chassis.chassis_ctrl_info.v_m_per_s)
                - wheel_K_R[4] * (chassis.leg_R.state_variable_feedback.phi - 0.0f)
                - wheel_K_R[5] * (chassis.leg_R.state_variable_feedback.phi_dot - 0.0f)
                ;

            chassis.leg_L.wheel_torque -= chassis.leg_L.wheel_torque_lqr;
            chassis.leg_R.wheel_torque += chassis.leg_R.wheel_torque_lqr;

        /* ============================= 转向pid 差速 ============================= */

            float target_yaw_speed = pid_calc(&chassis.chassis_turn_pos_pid,
                                              chassis.imu_reference.yaw_total_rad,
                                              chassis.chassis_ctrl_info.yaw_rad);

            /* ============================= 小陀螺 ============================= */

                // if(chassis.chassis_ctrl_mode == CHASSIS_SPIN)
                // {
                //     target_yaw_speed = SPIN_SPEED;
                // }

            chassis.wheel_turn_torque = pid_calc(&chassis.chassis_turn_speed_pid,
                                                  chassis.imu_reference.yaw_gyro,
                                                  target_yaw_speed);
        /* iss：待测试 */

            // chassis.leg_L.wheel_torque -= chassis.wheel_turn_torque;
            // chassis.leg_R.wheel_torque -= chassis.wheel_turn_torque;

        /* iss：测试 */

            // chassis.leg_L.wheel_torque = 0;
            // chassis.leg_R.wheel_torque = 0;

        /* ============================= 限幅 ============================= */

            VAL_LIMIT(chassis.leg_L.wheel_torque, MIN_WHEEL_TORQUE, MAX_WHEEL_TORQUE);
            VAL_LIMIT(chassis.leg_R.wheel_torque, MIN_WHEEL_TORQUE, MAX_WHEEL_TORQUE);
    }

/* ============================= 4. 计算力矩 关节 ============================= */

    static void joint_calc(void)
    {

        /* ============================= F ============================= */

            chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point = 0.0f;
            chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point = 0.0f;

            /* ============================= 变腿长 ============================= */

                /* ============================= 重力前馈 ============================= */

                    chassis.leg_L.torque_g =
                         chassis_physical_config.body_weight * GRAVITY * cosf(chassis.leg_L.state_variable_feedback.theta)/2.0f;

                    chassis.leg_R.torque_g =
                         chassis_physical_config.body_weight * GRAVITY * cosf(chassis.leg_R.state_variable_feedback.theta)/2.0f;

                    chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point += chassis.leg_L.torque_g;
                    chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point -= chassis.leg_R.torque_g;

                /* ============================= 腿长pid ============================= */

                    float L_L0_dot_set = pid_calc(&chassis.leg_L.leg_pos_pid,
                                                  chassis.leg_L.vmc.forward_kinematics.fk_L0.L0,
                                                  chassis.chassis_ctrl_info.target_length);

                    float R_L0_dot_set = pid_calc(&chassis.leg_R.leg_pos_pid,
                                                  chassis.leg_R.vmc.forward_kinematics.fk_L0.L0,
                                                  chassis.chassis_ctrl_info.target_length);

                    chassis.leg_L.leg_change_torque =  pid_calc(&chassis.leg_L.leg_speed_pid,
                                                            chassis.leg_L.vmc.forward_kinematics.fk_L0.L0_dot,
                                                                L_L0_dot_set);

                    chassis.leg_R.leg_change_torque =  pid_calc(&chassis.leg_R.leg_speed_pid,
                                                            chassis.leg_R.vmc.forward_kinematics.fk_L0.L0_dot,
                                                                R_L0_dot_set);

                    chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point += chassis.leg_L.leg_change_torque;
                    chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point -= chassis.leg_R.leg_change_torque;

            /* ============================= Roll pid ============================= */

                 chassis.roll_compensatory_torque = CHASSIS_ROLL_PID_P * (0.0f - chassis.imu_reference.roll_rad)
                                                   + CHASSIS_ROLL_PID_D * (0.0f - chassis.imu_reference.roll_gyro);

                VAL_LIMIT(chassis.roll_compensatory_torque, - CHASSIS_ROLL_PID_OUT_LIMIT, CHASSIS_ROLL_PID_OUT_LIMIT);

                // chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point += chassis.roll_compensatory_torque;
                // chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point -= chassis.roll_compensatory_torque;

            /* iss：测试 */

                // chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point = 0.0f;
                // chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point = 0.0f;

        /* ============================= Tp ============================= */

            chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point = 0;
            chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point = 0;

            /* ============================= 摆角力矩 ============================= */

                /* ============================= 3.1 不同腿长下进行多项式拟合 ============================= */

                    chassis_K_matrix_fitting(chassis.leg_L.vmc.forward_kinematics.fk_L0.L0, joint_K_L, joint_fitting_factor);
                    chassis_K_matrix_fitting(chassis.leg_R.vmc.forward_kinematics.fk_L0.L0, joint_K_R, joint_fitting_factor);

                /* ============================= 3.2 加入目标输入 ============================= */

                    chassis.leg_L.Tp_lqr =
                        - joint_K_L[0] * (chassis.leg_L.state_variable_feedback.theta - 0.0f)
                        - joint_K_L[1] * (chassis.leg_L.state_variable_feedback.theta_dot - 0.0f)
                        - joint_K_L[2] * (chassis.leg_L.state_variable_feedback.x - 0.0f)
                        - joint_K_L[3] * (chassis.leg_L.state_variable_feedback.x_dot - chassis.chassis_ctrl_info.v_m_per_s)
                        - joint_K_L[4] * (chassis.leg_L.state_variable_feedback.phi - 0.0f)
                        - joint_K_L[5] * (chassis.leg_L.state_variable_feedback.phi_dot - 0.0f)
                        ;

                    chassis.leg_R.Tp_lqr =
                        - joint_K_R[0] * (chassis.leg_R.state_variable_feedback.theta - 0.0f)
                        - joint_K_R[1] * (chassis.leg_R.state_variable_feedback.theta_dot - 0.0f)
                        - joint_K_R[2] * (chassis.leg_R.state_variable_feedback.x - 0.0f)
                        - joint_K_R[3] * (chassis.leg_R.state_variable_feedback.x_dot - chassis.chassis_ctrl_info.v_m_per_s)
                        - joint_K_R[4] * (chassis.leg_R.state_variable_feedback.phi - 0.0f)
                        - joint_K_R[5] * (chassis.leg_R.state_variable_feedback.phi_dot - 0.0f)
                        ;

            chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis.leg_L.Tp_lqr;
            chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point -= chassis.leg_R.Tp_lqr;

            /* ============================= 防劈叉pid ============================= */

                chassis.steer_compensatory_torque =
                    CHASSIS_LEG_COORDINATION_PID_P * (0.0f - chassis.phi0_error)
                    + CHASSIS_LEG_COORDINATION_PID_D * (0.0f - chassis.d_phi0_error);

                VAL_LIMIT(chassis.steer_compensatory_torque, -CHASSIS_LEG_COORDINATION_PID_OUT_LIMIT, CHASSIS_LEG_COORDINATION_PID_OUT_LIMIT);

                // chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis.steer_compensatory_torque;
                // chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point -= chassis.steer_compensatory_torque;

            /* iss：测试 */

                // chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point = 0;
                // chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point = 0;

            /* kss：不要在Tp层面修改左右侧旋转方向相反这个问题 因为解算出来加到电机上的力矩是对的 */

        /* =========================== 2. 正动力学变换 =========================== */

            vmc_forward_dynamics(&chassis.leg_L.vmc, &chassis_physical_config);
            vmc_forward_dynamics(&chassis.leg_R.vmc, &chassis_physical_config);

            /* =========================== 提取力矩 =========================== */

                chassis.leg_L.joint_F_torque = chassis.leg_L.vmc.forward_kinematics.T1_T4_set_point.E.T4_set_point;
                chassis.leg_L.joint_B_torque = chassis.leg_L.vmc.forward_kinematics.T1_T4_set_point.E.T1_set_point;

                chassis.leg_R.joint_F_torque = chassis.leg_R.vmc.forward_kinematics.T1_T4_set_point.E.T4_set_point;
                chassis.leg_R.joint_B_torque = chassis.leg_R.vmc.forward_kinematics.T1_T4_set_point.E.T1_set_point;

        /* ============================= 限幅 ============================= */

            VAL_LIMIT(chassis.leg_L.joint_F_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
            VAL_LIMIT(chassis.leg_L.joint_B_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
            VAL_LIMIT(chassis.leg_R.joint_F_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
            VAL_LIMIT(chassis.leg_R.joint_B_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);

    }

/* ============================= 倒地自救 ============================= */

    static float wrap_pi(float angle)
    {
        while (angle > PI)  angle -= 2.0f * PI;
        while (angle < -PI) angle += 2.0f * PI;
        return angle;
    }

    /* ============================= MIT给正v,8009逆时针旋转 ============================= */
    /* ============================= RF逆时针 大腿顺时针 phi1- RF要求逆时针 ============================= */
    /* ============================= RB逆时针 收小腿 phi4- ============================= */
    /* ============================= LF逆时针 大腿逆时针 phi1+ LF要求顺时针============================= */
    /* ============================= LB逆时针 放小腿 phi4+ ============================= */

    /* ============================= 恢复 ============================= */

        static void leg_reset_control(Leg *leg, float side_sign)
            {
                leg->vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point = 0.0f;
                leg->vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point = 0.0f;
                /* ============================= 收腿 ============================= */

                    // 目标腿长
                        float target_len_const = 0.18f;

                    // 腿长逐步收缩到最短
                        slope_following(&target_len_const, &leg->target_len, 0.0010f);

                    // 腿长 PID：位置环 → 速度环
                        float L0_dot_set = pid_calc(&leg->leg_pos_pid,
                                                     leg->vmc.forward_kinematics.fk_L0.L0,
                                                     leg->target_len);

                        float leg_len_torque = pid_calc(&leg->leg_speed_pid,
                                                         leg->vmc.forward_kinematics.fk_L0.L0_dot,
                                                         L0_dot_set);

                    // 收好腿
                        leg->leg_shortest_is_ready =
                        (fabsf(leg->vmc.forward_kinematics.fk_L0.L0 - target_len_const) < 0.010f);

                /* ============================= 转腿 ============================= */

                        leg->leg_theta_reset_pid.iout = 0.0f;

                        // 目标角
                            leg->target_theta = 0.0f;

                        // 摆角逐步过渡到目标角
                            slope_following(&leg->target_theta, &leg->ramp_target_theta, (0.25f * 2.0f * PI * 0.001f));

                        // theta PID
                            float theta_torque = pid_calc(&leg->leg_theta_reset_pid,
                                                       leg->state_variable_feedback.theta,
                                                       leg->ramp_target_theta);

                        // 转好角
                            float theta_err = wrap_pi(leg->state_variable_feedback.theta - leg->target_theta);
                            leg->leg_pos_is_ready =
                            (fabsf(theta_err) < (5.0f * DEGREE_TO_RAD));

                /* ============================= VMC 正动力学转换 ============================= */

                    leg->vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point = side_sign * leg_len_torque;
                    leg->vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point = side_sign * theta_torque;

                    vmc_forward_dynamics(&leg->vmc, &chassis_physical_config);

                    leg->joint_F_torque = leg->vmc.forward_kinematics.T1_T4_set_point.E.T4_set_point;
                    leg->joint_B_torque = leg->vmc.forward_kinematics.T1_T4_set_point.E.T1_set_point;

            /* ============================= 单腿自救好 ============================= */

                leg->leg_selfhelp_ready =
                    (leg->leg_shortest_is_ready == true) && (leg->leg_pos_is_ready == true);

            }

    /* ============================= 倒地自救 ============================= */

        static void ready_to_balance(void)
        {
            if (chassis.leg_L.leg_selfhelp_ready == false || chassis.leg_R.leg_selfhelp_ready == false)
            {
                chassis.leg_L.wheel_torque = 0.0f;
                chassis.leg_R.wheel_torque = 0.0f;

                leg_reset_control(&chassis.leg_L, +1.0f);
                leg_reset_control(&chassis.leg_R, -1.0f);

                chassis.chassis_selfhelp_finish = false;
            }
            else
            {
                chassis.chassis_selfhelp_finish = true;
            }
        }

/**********************************************************************************
 *                                  底盘任务                                       *
 *********************************************************************************/

/* =========================== 底盘失能任务 ============================= */

    static void chassis_disable_task(void)
    {
        /* =========================== 数据处理 供下次遥控============================= */

            /* =========================== x清零 ============================= */

            chassis.leg_L.state_variable_feedback.x = 0.0f;
            chassis.leg_R.state_variable_feedback.x = 0.0f;

            /* ===========================  储存yaw 供下次控制转向 ============================= */

            chassis.chassis_ctrl_info.yaw_rad = chassis.imu_reference.yaw_total_rad;

            /* =========================== 下次上电初始腿长 ============================= */

            chassis.chassis_ctrl_info.target_length = MIN_L0;

        /* =========================== 标志位 ============================= */

            /* =========================== 底盘初始化标志位置0 ============================= */

                chassis.init_flag = false;

            /* =========================== 倒地自救标志位置0 ============================= */

                chassis.leg_L.leg_shortest_is_ready = false;
                chassis.leg_L.leg_pos_is_ready = false;
                chassis.leg_L.leg_selfhelp_ready = false;

                chassis.leg_R.leg_shortest_is_ready = false;
                chassis.leg_R.leg_pos_is_ready = false;
                chassis.leg_R.leg_selfhelp_ready = false;

                chassis.chassis_selfhelp_finish = false;

                chassis.chassis_ready_to_balance = false;

        /* =========================== 失能 ============================= */

            /* =========================== 轮毂失能 ============================= */

                chassis.leg_L.wheel_torque = 0;
                chassis.leg_R.wheel_torque = 0;

            /* =========================== 关节失能 ============================= */

                chassis.leg_L.joint_F_torque = 0;
                chassis.leg_L.joint_B_torque = 0;
                chassis.leg_R.joint_F_torque = 0;
                chassis.leg_R.joint_B_torque = 0;
                chassis.leg_L.F_speed = 0;
                chassis.leg_L.B_speed = 0;
                chassis.leg_R.F_speed = 0;
                chassis.leg_R.B_speed = 0;
                chassis.leg_L.kd = 0;
                chassis.leg_R.kd = 0;
                joint_disable();


            /* =========================== 进入失能模式 ============================= */

                chassis.chassis_ctrl_mode = CHASSIS_DISABLE;
    }

/* =========================== 底盘初始化任务 ============================= */

    static void chassis_init_task(void)
    {
        /* =========================== 使能关节 ============================= */

            joint_enable();

        /* =========================== 底盘初始化标志位置1 ============================= */

            chassis.init_flag = true;

    }

/* ============================= 底盘使能任务 ============================= */

    static void chassis_enable_task(void)
    {

        /* ============================= 4. 计算力矩 轮毂 ============================= */

            wheel_calc();

        /* ============================= 4. 计算力矩 关节 ============================= */

            // joint_enable();
            joint_calc();

        /* ============================= 倒地自救 ============================= */

            if (chassis.chassis_ready_to_balance == false)
            {
                ready_to_balance();
            }
    }

/* =========================== 发送力矩 =========================== */

    static void MIT_send_torque_task
        (   float joint_LF_torque, float joint_LB_torque,
            float joint_RF_torque, float joint_RB_torque,
            float wheel_L_torque, float wheel_R_torque,
            float LF_speed,float LB_speed, float L_Kd,
            float RF_speed,float RB_speed, float R_Kd)
    {

        /* =========================== 关节8009P电机 =========================== */

                /* =========================== 左关节8009P电机 MIT模式 =========================== */

                set_left_dm8009p_MIT(&joint[LF], 0.0f, LF_speed, 0.0f, L_Kd, joint_LF_torque);
                set_left_dm8009p_MIT(&joint[LB], 0.0f, LB_speed, 0.0f, L_Kd, joint_LB_torque);

                /* =========================== 右关节8009P电机 MIT模式 =========================== */

                set_right_dm8009p_MIT(&joint[RF], 0.0f, RF_speed, 0.0f, R_Kd, joint_RF_torque);
                set_right_dm8009p_MIT(&joint[RB], 0.0f, RB_speed, 0.0f, R_Kd, joint_RB_torque);

        /* =========================== 轮毂3508电机 =========================== */

            /* =========================== 根据比例减速 =========================== */

                int16_t L_wheel_data = wheel_L_torque / TORQUE_CONSTANT_3508 * DATA_PER_A;
                int16_t R_wheel_data = wheel_R_torque / TORQUE_CONSTANT_3508 * DATA_PER_A;

            /* =========================== DJI电机设置扭矩电流 =========================== */

                DJI_Current_Set(L_wheel_data,R_wheel_data,0,0);

    }

/*****************************************************************************
 *                               rtos任务                                    *
 ****************************************************************************/

/* =========================== 底盘初始化任务 ============================= */

    void chassis_init(void)
    {

        /* =========================== 初始化底盘模式为失能 ============================= */

            chassis.chassis_ctrl_mode = CHASSIS_DISABLE;

        /* =========================== 关节电机ID初始化 ============================= */

            joint_init();

        /* =========================== 轮毂电机初始化 ============================= */

            wheel_init();

        /* =========================== 底盘pid初始化 ============================= */

            chassis_pid_init();

        /* =========================== 姿态解算初始化 ============================= */

            INS_Init();

        /* =========================== 滤波器初始化 ============================= */

            /* =========================== θ θ'融合观测器初始化 ============================= */

                Theta_EstimateKF_Init(&Theta_EstimateKF_LegL);
                Theta_EstimateKF_Init(&Theta_EstimateKF_LegR);

            /* =========================== θ''低通滤波器初始化 ============================= */

                low_pass_filter_init(&chassis.leg_L.theta_ddot, 0.75f);
                low_pass_filter_init(&chassis.leg_R.theta_ddot, 0.75f);

            /* =========================== x' x''融合观测器初始化 ============================= */

                L0_EstimateKF_Init(&Speed_EstimateKF);

            /* =========================== x''低通滤波器初始化 ============================= */

                low_pass_filter_init(&chassis.leg_L.x_ddot, 0.75f);
                low_pass_filter_init(&chassis.leg_R.x_ddot, 0.75f);

            /* =========================== L0 L0'融合观测器初始化 ============================= */

                L0_EstimateKF_Init(&L0_EstimateKF);

            /* =========================== L0''低通滤波器初始化 ============================= */

                low_pass_filter_init(&chassis.leg_L.L0_ddot, 0.75f);
                low_pass_filter_init(&chassis.leg_R.L0_ddot, 0.75f);

            /* =========================== φ0 φ0'融合观测器初始化 ============================= */

                PHI0_EstimateKF_Init(&PHI0_EstimateKF);

            /* =========================== φ0''低通滤波器初始化 ============================= */

                low_pass_filter_init(&chassis.leg_L.phi0_ddot, 0.75f);
                low_pass_filter_init(&chassis.leg_R.phi0_ddot, 0.75f);

            /* =========================== φ4 φ4'融合观测器初始化 ============================= */

                // PHI4_EstimateKF_Init(&PHI4_EstimateKF);

        /* =========================== x归零 ============================= */

            chassis.leg_L.state_variable_feedback.x = 0;
            chassis.leg_R.state_variable_feedback.x = 0;

    }

/* =========================== 底盘任务 =========================== */

    void chassis_task(void)
    {

        /* =========================== 获取底盘遥控器信息 =========================== */

            chassis_remote_cmd();

        /* =========================== 底盘观测器更新 =========================== */

            chassis_observer_update();

        /* =========================== 根据拨杆设置任务模式 =========================== */

            switch (chassis.chassis_ctrl_mode)
            {

            /* =========================== 失能模式 进入底盘失能任务 ============================= */

                case CHASSIS_DISABLE:
                {
                    chassis_disable_task();
                    break;
                }

            /* =========================== 初始化模式 进入底盘初始化任务 ============================= */

                case CHASSIS_INIT:
                {
                    chassis_init_task();
                    break;
                }

            /* =========================== 使能或者小陀螺模式 进入底盘使能任务 =========================== */

                case CHASSIS_ENABLE:
                case CHASSIS_SPIN:
                {
                    chassis_enable_task();
                    break;
                }

                default:
                {
                    break;
                }
            }

        /* =========================== 发送力矩 =========================== */

            /* iss：测试 */

                // chassis.leg_L.joint_B_torque = 0.0f;
                // chassis.leg_L.joint_F_torque = 0.0f;
                // chassis.leg_R.joint_B_torque = 0.0f;
                // chassis.leg_R.joint_F_torque = 0.0f;

                // chassis.leg_L.wheel_torque = 0;
                // chassis.leg_R.wheel_torque = 0;

               // chassis.leg_R.F_speed = 0;
               // chassis.leg_R.B_speed = 0;

               // chassis.leg_L.F_speed = 5;
               // chassis.leg_L.B_speed = 5;
               // chassis.leg_L.kd = 2;
               // chassis.leg_R.kd = 2;

            MIT_send_torque_task(chassis.leg_L.joint_F_torque,
                                 chassis.leg_L.joint_B_torque,
                                 chassis.leg_R.joint_F_torque,
                                 chassis.leg_R.joint_B_torque,
                                 chassis.leg_L.wheel_torque,
                                 chassis.leg_R.wheel_torque,
                                 chassis.leg_L.F_speed,chassis.leg_L.B_speed,chassis.leg_L.kd,
                                 chassis.leg_R.F_speed,chassis.leg_R.B_speed,chassis.leg_R.kd);
            /* iss：测试 */

                // MIT_send_torque_task(0,0,0,0,0,0,0,0,0,0,0,0);

        /* iss：测试 */

            USART_Vofa_Justfloat_Transmit(
            chassis.imu_reference.robot_ax,
            chassis.leg_L.state_variable_feedback.x_dot_raw,
            chassis.leg_L.w_ecd,
            chassis.leg_L.phi_bc_dot,
            chassis.leg_L.w_eb,
            chassis.leg_L.v_b,
            chassis.leg_L.state_variable_feedback.theta,
            chassis.leg_L.vmc.forward_kinematics.fk_L0.L0_dot,0
            );

            // chassis.leg_L.state_variable_feedback.theta,chassis.leg_L.state_variable_feedback.theta_dot
    //             ,chassis.leg_L.state_variable_feedback.x,chassis.leg_L.state_variable_feedback.x_dot
    //         ,chassis.leg_L.state_variable_feedback.phi,chassis.leg_L.state_variable_feedback.phi_dot
    //
    //         ,chassis.leg_R.state_variable_feedback.theta,chassis.leg_R.state_variable_feedback.theta_dot
    //             ,chassis.leg_R.state_variable_feedback.x,chassis.leg_R.state_variable_feedback.x_dot
    //         ,chassis.leg_R.state_variable_feedback.phi,chassis.leg_R.state_variable_feedback.phi_dot,
    //
    //     joint_K_L[0] ,joint_K_L[1] ,joint_K_L[2] ,joint_K_L[3] ,joint_K_L[4] ,joint_K_L[5] ,
    //
    // - joint_K_L[0] * (chassis.leg_L.state_variable_feedback.theta - 0.0f),
    // - joint_K_L[1] * (chassis.leg_L.state_variable_feedback.theta_dot - 0.0f),
    // - joint_K_L[2] * (chassis.leg_L.state_variable_feedback.x - 0.0f),
    // - joint_K_L[3] * (chassis.leg_L.state_variable_feedback.x_dot - chassis.chassis_ctrl_info.v_m_per_s),
    // - joint_K_L[4] * (chassis.leg_L.state_variable_feedback.phi - 0.0f),
    // - joint_K_L[5] * (chassis.leg_L.state_variable_feedback.phi_dot - 0.0f),
    //
    // joint_K_R[0] ,joint_K_R[1] ,joint_K_R[2] ,joint_K_R[3] ,joint_K_R[4] ,joint_K_R[5] ,
    // - joint_K_R[0] * (chassis.leg_R.state_variable_feedback.theta - 0.0f),
    // - joint_K_R[1] * (chassis.leg_R.state_variable_feedback.theta_dot - 0.0f),
    // - joint_K_R[2] * (chassis.leg_R.state_variable_feedback.x - 0.0f),
    // - joint_K_R[3] * (chassis.leg_R.state_variable_feedback.x_dot - chassis.chassis_ctrl_info.v_m_per_s),
    // - joint_K_R[4] * (chassis.leg_R.state_variable_feedback.phi - 0.0f),
    // - joint_K_R[5] * (chassis.leg_R.state_variable_feedback.phi_dot - 0.0f),
    //
    // wheel_K_L[0],wheel_K_L[1],wheel_K_L[2],wheel_K_L[3],wheel_K_L[4],wheel_K_L[5],
    //
    //            - wheel_K_L[0] * (chassis.leg_L.state_variable_feedback.theta - 0.0f),
    //            - wheel_K_L[1] * (chassis.leg_L.state_variable_feedback.theta_dot - 0.0f),
    //            - wheel_K_L[2] * (chassis.leg_L.state_variable_feedback.x - 0.0f),
    //            - wheel_K_L[3] * (chassis.leg_L.state_variable_feedback.x_dot - chassis.chassis_ctrl_info.v_m_per_s),
    //            - wheel_K_L[4] * (chassis.leg_L.state_variable_feedback.phi - 0.0f),
    //            - wheel_K_L[5] * (chassis.leg_L.state_variable_feedback.phi_dot - 0.0f),
    //
    //            wheel_K_R[0],wheel_K_R[1],wheel_K_R[2],wheel_K_R[3],wheel_K_R[4],wheel_K_R[5],
    //
    //            - wheel_K_R[0] * (chassis.leg_R.state_variable_feedback.theta - 0.0f),
    //            - wheel_K_R[1] * (chassis.leg_R.state_variable_feedback.theta_dot - 0.0f),
    //            - wheel_K_R[2] * (chassis.leg_R.state_variable_feedback.x - 0.0f),
    //            - wheel_K_R[3] * (chassis.leg_R.state_variable_feedback.x_dot - chassis.chassis_ctrl_info.v_m_per_s),
    //            - wheel_K_R[4] * (chassis.leg_R.state_variable_feedback.phi - 0.0f),
    //            - wheel_K_R[5] * (chassis.leg_R.state_variable_feedback.phi_dot - 0.0f)

            // //
            // ,chassis.imu_reference.yaw_rad
            // ,chassis.imu_reference.yaw_total_rad
            // ,chassis.imu_reference.pitch_rad
            // ,chassis.imu_reference.roll_rad
            // ,chassis.chassis_ctrl_info.yaw_rad
            // ,chassis.wheel_turn_torque
            //
            //     ,chassis.leg_L.vmc.forward_kinematics.fk_L0.L0,chassis.leg_R.vmc.forward_kinematics.fk_L0.L0,
            //     chassis.chassis_ctrl_info.v_m_per_s,
            //     - wheel_K_L[0] * (-chassis.leg_L.state_variable_feedback.theta - 0.0f),
            //                     - wheel_K_L[1] * (-chassis.leg_L.state_variable_feedback.theta_dot - 0.0f),
            //                     - wheel_K_L[2] * (-chassis.leg_L.state_variable_feedback.x - 0.0f),
            //                     - wheel_K_L[3] * (-chassis.leg_L.state_variable_feedback.x_dot - chassis.chassis_ctrl_info.v_m_per_s),
                    //             - wheel_K_L[4] * (-chassis.leg_L.state_variable_feedback.phi - 0.0f),
                    //             - wheel_K_L[5] * (-chassis.leg_L.state_variable_feedback.phi_dot - 0.0f)
                    //             ,- wheel_K_R[0] * (-chassis.leg_R.state_variable_feedback.theta - 0.0f),
                    // - wheel_K_R[1] * (-chassis.leg_R.state_variable_feedback.theta_dot - 0.0f),
                    // - wheel_K_R[2] * (-chassis.leg_R.state_variable_feedback.x - 0.0f),
                    // - wheel_K_R[3] * (-chassis.leg_R.state_variable_feedback.x_dot - chassis.chassis_ctrl_info.v_m_per_s),
                    // - wheel_K_R[4] * (-chassis.leg_R.state_variable_feedback.phi - 0.0f),
                    // - wheel_K_R[5] * (-chassis.leg_R.state_variable_feedback.phi_dot - 0.0f)


                // ,chassis.leg_L.wheel_torque
                // ,chassis.leg_R.wheel_torque


                // ,chassis.leg_L.state_variable_feedback.alpha,chassis.leg_R.state_variable_feedback.alpha,0,0
                // ,chassis.leg_L.state_variable_feedback.theta_dot,chassis.leg_R.state_variable_feedback.theta_dot,chassis.leg_L.leg_change_torque,chassis.leg_R.leg_change_torque
                // ,0,0,0,0);
                //
                // ,chassis.leg_L.F_speed,chassis.leg_L.B_speed
                // ,chassis.leg_R.F_speed,chassis.leg_R.B_speed
                //
                // ,chassis.leg_L.vmc.forward_kinematics.fk_phi.phi1,chassis.leg_L.vmc.forward_kinematics.fk_phi.phi4,chassis.leg_L.vmc.forward_kinematics.fk_phi.phi0
                // ,chassis.leg_R.vmc.forward_kinematics.fk_phi.phi1,chassis.leg_R.vmc.forward_kinematics.fk_phi.phi4,chassis.leg_R.vmc.forward_kinematics.fk_phi.phi0
                //
                // ,(chassis.leg_L.state_variable_feedback.theta - THETA_OFFSET_L_CON)-(chassis.leg_R.state_variable_feedback.theta - THETA_OFFSET_R_CON),0,0,0,
                // 0,wheel_K_L[0], wheel_K_L[4],wheel_K_R[0], wheel_K_R[4] );

                // (get_joint_motors() + 0)->pos_r,(get_joint_motors() + 1)->pos_r,(get_joint_motors() + 2)->pos_r,(get_joint_motors() + 3)->pos_r
                // , (get_joint_motors() + 0)->angular_vel, (get_joint_motors() + 1)->angular_vel, (get_joint_motors() + 2)->angular_vel, (get_joint_motors() + 3)->angular_vel
                // ,0);

                // ,chassis.leg_L.ready_to_balance,chassis.leg_L.leg_is_shortest,chassis.leg_L.leg_recover_finish
                // ,chassis.leg_R.ready_to_balance,chassis.leg_R.leg_is_shortest,chassis.leg_R.leg_recover_finish
                // ,chassis.chassis_recover_finish
                //
                // ,chassis.leg_L.wheel_torque_lqr,chassis.leg_R.wheel_torque_lqr);

            //
            // USART_Vofa_Justfloat_Transmit(
            //     chassis.leg_L.state_variable_feedback.theta,chassis.leg_L.state_variable_feedback.theta_dot
            //     ,chassis.leg_L.state_variable_feedback.x,chassis.leg_L.state_variable_feedback.x_dot
            //     ,chassis.leg_L.state_variable_feedback.phi,chassis.leg_L.state_variable_feedback.phi_dot
            //
            //     ,chassis.leg_R.state_variable_feedback.theta,chassis.leg_R.state_variable_feedback.theta_dot
            //     ,chassis.leg_R.state_variable_feedback.x,chassis.leg_R.state_variable_feedback.x_dot
            //     ,chassis.leg_R.state_variable_feedback.phi,chassis.leg_R.state_variable_feedback.phi_dot
            //
            //     ,chassis.leg_L.wheel_torque_lqr,chassis.leg_R.wheel_torque_lqr,
            //
            //     chassis.leg_L.w_ecd,chassis.leg_L.phi_bc_dot,chassis.leg_L.w_eb,
            //     chassis.leg_L.w,chassis.leg_L.v,
            //     chassis.leg_L.vmc.forward_kinematics.fk_L0.L0,chassis.leg_L.vmc.forward_kinematics.fk_L0.L0_dot,
            //     chassis.leg_L.v_b,chassis.leg_L.state_variable_feedback.x_dot_raw
            //
            //     ,chassis.leg_R.w_ecd,chassis.leg_R.phi_bc_dot
            //
            // ,joint[LF].Temp_mos,joint[LF].Temp_motor,joint[LF].Error,joint[LB].Temp_mos,joint[LB].Temp_motor,joint[LB].Error
            // ,joint[RF].Temp_mos,joint[RF].Temp_motor,joint[RF].Error,joint[RB].Temp_mos,joint[RB].Temp_motor,joint[RB].Error
            // ,joint[LF].Error,joint[LB].Error,joint[RF].Error,joint[RB].Error
            // chassis.chassis_ctrl_mode,
            //
            // chassis.leg_L.leg_is_shortest,
            // chassis.leg_L.leg_ready_to_selfhelp,
            // chassis.leg_L.leg_selfhelp_finish,
            //
            // chassis.leg_R.leg_is_shortest,
            // chassis.leg_R.leg_ready_to_selfhelp ,
            // chassis.leg_R.leg_selfhelp_finish,
            //
            // chassis.chassis_selfhelp_finish ,
            //
            // chassis.chassis_ready_to_balance,
            //
            // chassis.leg_L.wheel_torque,chassis.leg_R.wheel_torque
            // );



    }
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


/** 底盘pid初始化 **/
static void chassis_pid_init(void) {

    /** Wheel **/
    // 转向PID
    pid_init(&chassis.chassis_turn_pos_pid,
             CHASSIS_TURN_POS_PID_OUT_LIMIT,
             CHASSIS_TURN_POS_PID_IOUT_LIMIT,
             CHASSIS_TURN_POS_PID_P,
             CHASSIS_TURN_POS_PID_I,
             CHASSIS_TURN_POS_PID_D);

    pid_init(&chassis.chassis_turn_speed_pid,
             CHASSIS_TURN_SPEED_PID_OUT_LIMIT,
             CHASSIS_TURN_SPEED_PID_IOUT_LIMIT,
             CHASSIS_TURN_SPEED_PID_P,
             CHASSIS_TURN_SPEED_PID_I,
             CHASSIS_TURN_SPEED_PID_D);

    /** Joint **/
    // 防劈叉PID
    pid_init(&chassis.chassis_leg_coordination_pid,
             CHASSIS_LEG_COORDINATION_PID_OUT_LIMIT,
             CHASSIS_LEG_COORDINATION_PID_IOUT_LIMIT,
             CHASSIS_LEG_COORDINATION_PID_P,
             CHASSIS_LEG_COORDINATION_PID_I,
             CHASSIS_LEG_COORDINATION_PID_D);

    // 腿长位置环PID
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

    // 腿长速度环PID
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

    // Roll补偿PID
    pid_init(&chassis.chassis_roll_pid,
             CHASSIS_ROLL_PID_OUT_LIMIT,
             CHASSIS_ROLL_PID_IOUT_LIMIT,
             CHASSIS_ROLL_PID_P,
             CHASSIS_ROLL_PID_I,
             CHASSIS_ROLL_PID_D);
}

/** 底盘初始化 **/
void chassis_init(void) {
    /** 初始化底盘模式 **/
    chassis.chassis_ctrl_mode = CHASSIS_DISABLE;

    /** 关节电机初始化 **/
    joint_init();

    /** 轮毂电机初始化 **/
    wheel_init();

    /** 底盘pid初始化 **/
    chassis_pid_init();

    /** 姿态解算初始化 **/
    INS_Init();

    /** 滤波器初始化 **/
    // 卡尔曼滤波
    Speed_EstimateKF_Init(&Speed_EstimateKF); // 速度-加速度融合观测器
    Theta_EstimateKF_Init(&Theta_EstimateKF); // 摆角-角速度融合观测器

}

/** 底盘接收遥控器信息 **/
static void set_chassis_ctrl_info(void) {

    /** 期望速度 **/
    chassis.chassis_ctrl_info.v_m_per_s = (float) (remote_ctrl.rc.ch[CHASSIS_VX_CHANNEL]) * RC_TO_VX;;

    /** 转向 **/
    chassis.chassis_ctrl_info.yaw_rad -= (float) (remote_ctrl.rc.ch[CHASSIS_YAW_CHANNEL]) * (-RC_TO_YAW_INCREMENT);

    /** 期望腿长 **/
    chassis.chassis_ctrl_info.target_length = MID_L0;

}

/** 底盘根据遥控器设置模式 **/
static void set_chassis_mode(void) {

    if (switch_is_down(remote_ctrl.rc.s[RC_s_R])) { // 失能
        chassis.chassis_last_ctrl_mode = chassis.chassis_ctrl_mode;
        chassis.chassis_ctrl_mode = CHASSIS_DISABLE;
    } else if (switch_is_mid(remote_ctrl.rc.s[RC_s_R]) && (chassis.init_flag == false)) { // 初始化模式
        chassis.chassis_last_ctrl_mode = chassis.chassis_ctrl_mode;
        chassis.chassis_ctrl_mode = CHASSIS_INIT;
    } else if (switch_is_mid(remote_ctrl.rc.s[RC_s_R]) && (chassis.init_flag == true)) { // 使能
        chassis.chassis_last_ctrl_mode = chassis.chassis_ctrl_mode;
        chassis.chassis_ctrl_mode = CHASSIS_ENABLE;
    } else if (switch_is_up(remote_ctrl.rc.s[RC_s_R])) {
        chassis.chassis_last_ctrl_mode = chassis.chassis_ctrl_mode;
        chassis.chassis_ctrl_mode = CHASSIS_SPIN;
    }

}

void chassis_remote_cmd(void) {
    set_chassis_mode();

    set_chassis_ctrl_info();
}

/** 底盘倒地自救 **/
static void chassis_selfhelp(void) {
    /** 检测是否需要自救 **/



    /** 倒地自救尚未完成 **/

}

/** 获取底盘传感器数据 **/
static void get_IMU_info(void) {

    /** Yaw **/
    chassis.imu_reference.yaw_rad = -DM_IMU.yaw * DEGREE_TO_RAD;

    if (chassis.imu_reference.yaw_rad - chassis.imu_reference.yaw_last_rad > PI) {
        chassis.imu_reference.yaw_round_count--;
    } else if (chassis.imu_reference.yaw_rad - chassis.imu_reference.yaw_last_rad < -PI) {
        chassis.imu_reference.yaw_round_count++;
    }

    chassis.imu_reference.yaw_total_rad =
            2 * PI * chassis.imu_reference.yaw_round_count + chassis.imu_reference.yaw_rad;
    chassis.imu_reference.yaw_last_rad = chassis.imu_reference.yaw_rad;

    /** Pitch **/
    chassis.imu_reference.pitch_rad = -DM_IMU.roll * DEGREE_TO_RAD;

    /** Roll **/
    chassis.imu_reference.roll_rad = -DM_IMU.pitch * DEGREE_TO_RAD;

    /** 更新各轴加速度和角速度 **/
    // rad/s
    chassis.imu_reference.pitch_gyro = -DM_IMU.gyro[X];
    chassis.imu_reference.yaw_gyro = -DM_IMU.gyro[Z];
    chassis.imu_reference.roll_gyro = -DM_IMU.gyro[Y];

    chassis.imu_reference.ax = -DM_IMU.accel[Y];
    chassis.imu_reference.ay = DM_IMU.accel[X];
    chassis.imu_reference.az = DM_IMU.accel[Z];

    /** 将机体坐标系的加速度转换为世界坐标系 **/
    Body_Accel_To_Earth();

}

/** 更新底盘变量 **/
static void chassis_variable_update(void) {

    /********************* phi phi_dot ***********************/
    chassis.leg_L.state_variable_feedback.phi = chassis.imu_reference.pitch_rad;
    chassis.leg_R.state_variable_feedback.phi = chassis.imu_reference.pitch_rad;

    chassis.leg_L.state_variable_feedback.phi_dot = chassis.imu_reference.pitch_gyro;
    chassis.leg_R.state_variable_feedback.phi_dot = chassis.imu_reference.pitch_gyro;

    /********************* theta theta_dot ***********************/

    //1.theta
    float L_theta_raw = PI / 2 - (chassis.leg_L.vmc.forward_kinematics.fk_phi.phi0 + chassis.imu_reference.pitch_rad);
    float R_theta_raw = PI / 2 - (chassis.leg_R.vmc.forward_kinematics.fk_phi.phi0 + chassis.imu_reference.pitch_rad);

    //2. theta_dot

    chassis.leg_L.state_variable_feedback.theta_dot_last = chassis.leg_L.state_variable_feedback.theta_dot;
    chassis.leg_R.state_variable_feedback.theta_dot_last = chassis.leg_R.state_variable_feedback.theta_dot;

    chassis.leg_L.state_variable_feedback.theta_dot = -(chassis.leg_L.vmc.forward_kinematics.fk_phi.d_phi0 +
                                                        chassis.imu_reference.pitch_gyro);
    chassis.leg_R.state_variable_feedback.theta_dot = -(chassis.leg_R.vmc.forward_kinematics.fk_phi.d_phi0 +
                                                        chassis.imu_reference.pitch_gyro);

    theta_calc(&chassis.leg_L, L_theta_raw, chassis.leg_L.state_variable_feedback.theta_dot);
    theta_calc(&chassis.leg_R, R_theta_raw, chassis.leg_R.state_variable_feedback.theta_dot);

    // 2.1 theta_ddot 需要加低通滤波
    chassis.leg_L.state_variable_feedback.theta_ddot =
            (chassis.leg_L.state_variable_feedback.theta_dot - chassis.leg_L.state_variable_feedback.theta_dot_last) /
            (CHASSIS_PERIOD * 0.001f);
    chassis.leg_R.state_variable_feedback.theta_ddot =
            (chassis.leg_R.state_variable_feedback.theta_dot - chassis.leg_R.state_variable_feedback.theta_dot_last) /
            (CHASSIS_PERIOD * 0.001f);

}

/** 计算驱动轮力矩 **/
static void wheel_calc(void) {
    /******************************* Wheel *************************************/

    /** 根据腿长和三次拟合系数拟合出反馈增益K **/
    chassis_K_matrix_fitting(chassis.leg_L.vmc.forward_kinematics.fk_L0.L0, wheel_K_L, wheel_fitting_factor);
    chassis_K_matrix_fitting(chassis.leg_R.vmc.forward_kinematics.fk_L0.L0, wheel_K_R, wheel_fitting_factor);

    float target_yaw_speed = pid_calc(&chassis.chassis_turn_pos_pid,
                                      chassis.imu_reference.yaw_total_rad,
                                      chassis.chassis_ctrl_info.yaw_rad);

    chassis.wheel_turn_torque = -pid_calc(&chassis.chassis_turn_speed_pid,
                                          chassis.imu_reference.yaw_gyro,
                                          target_yaw_speed);

    chassis.leg_L.wheel_torque = 0.0f;
    chassis.leg_R.wheel_torque = 0.0f;


    VAL_LIMIT(chassis.leg_L.wheel_torque, MIN_WHEEL_TORQUE, MAX_WHEEL_TORQUE);
    VAL_LIMIT(chassis.leg_R.wheel_torque, MIN_WHEEL_TORQUE, MAX_WHEEL_TORQUE);

}

/** 计算关节力矩 **/
static void joint_calc(void) {
/******************************* Joint *************************************/

    chassis_K_matrix_fitting(chassis.leg_L.vmc.forward_kinematics.fk_L0.L0, joint_K_L, joint_fitting_factor);
    chassis_K_matrix_fitting(chassis.leg_R.vmc.forward_kinematics.fk_L0.L0, joint_K_R, joint_fitting_factor);

    /** Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp **/

    /****** 防劈叉pid ******/
    chassis.steer_compensatory_torque = CHASSIS_LEG_COORDINATION_PID_P * (0.0f - chassis.phi0_error)
                                        + CHASSIS_LEG_COORDINATION_PID_D * (0.0f - chassis.d_phi0_error); // 注意微分项正负


    chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point = 0.0f;
    chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point = 0.0f;

    /** 不要在Tp层面修改左右侧旋转方向相反这个问题 因为解算出来加到电机上的力矩是对的 **/

    /** End End End End End End End End End End End End End End End End End End End End End End End End **/


    /** F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F **/

    /****** Leg pid ******/

    float L_L0_dot_set = pid_calc(&chassis.leg_L.leg_pos_pid,
                                  chassis.leg_L.vmc.forward_kinematics.fk_L0.L0,
                                  chassis.chassis_ctrl_info.target_length);

    float R_L0_dot_set = pid_calc(&chassis.leg_R.leg_pos_pid,
                                  chassis.leg_R.vmc.forward_kinematics.fk_L0.L0,
                                  chassis.chassis_ctrl_info.target_length);

    pid_calc(&chassis.leg_L.leg_speed_pid,
             chassis.leg_L.vmc.forward_kinematics.fk_L0.L0_dot,
             L_L0_dot_set);

    pid_calc(&chassis.leg_R.leg_speed_pid,
             chassis.leg_R.vmc.forward_kinematics.fk_L0.L0_dot,
             R_L0_dot_set);


    /****** Roll pid ******/

    chassis.roll_compensatory_torque = CHASSIS_ROLL_PID_P * (0.0f - chassis.imu_reference.roll_rad)
                                       + CHASSIS_ROLL_PID_D * (0.0f - chassis.imu_reference.roll_gyro); // 注意微分项正负


    chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point = 0.0f;
    chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point = 0.0f;

    /** End End End End End End End End End End End End End End End End End End End End End End End End **/

    // 计算关节电机力矩
    vmc_forward_dynamics(&chassis.leg_L.vmc, &chassis_physical_config);
    vmc_forward_dynamics(&chassis.leg_R.vmc, &chassis_physical_config);

    chassis.leg_L.joint_F_torque = chassis.leg_L.vmc.forward_kinematics.T1_T4_set_point.E.T1_set_point;//F
    chassis.leg_L.joint_B_torque = chassis.leg_L.vmc.forward_kinematics.T1_T4_set_point.E.T4_set_point;//B

    chassis.leg_R.joint_F_torque = chassis.leg_R.vmc.forward_kinematics.T1_T4_set_point.E.T1_set_point;//F
    chassis.leg_R.joint_B_torque = chassis.leg_R.vmc.forward_kinematics.T1_T4_set_point.E.T4_set_point;//B

    // 输出限幅
    VAL_LIMIT(chassis.leg_L.joint_F_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
    VAL_LIMIT(chassis.leg_L.joint_B_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
    VAL_LIMIT(chassis.leg_R.joint_F_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
    VAL_LIMIT(chassis.leg_R.joint_B_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
}

/*******************************************************************************
 *                                  Task                                       *
 *******************************************************************************/

/** 底盘失能任务 **/
static void chassis_disable_task(void) {

    chassis.leg_L.wheel_torque = 0;
    chassis.leg_R.wheel_torque = 0;

    chassis.leg_L.joint_F_torque = 0;
    chassis.leg_L.joint_B_torque = 0;
    chassis.leg_R.joint_F_torque = 0;
    chassis.leg_R.joint_B_torque = 0;

    chassis.chassis_ctrl_mode = CHASSIS_DISABLE;

    chassis.leg_L.state_variable_feedback.x = 0.0f;
    chassis.leg_R.state_variable_feedback.x = 0.0f;

    chassis.chassis_ctrl_info.yaw_rad = chassis.imu_reference.yaw_total_rad;

    chassis.chassis_ctrl_info.target_length = MIN_L0;

    /** 初始化标志位 **/

    // 底盘初始化标志位
    chassis.init_flag = false;

    chassis.chassis_recover_finish = false;

}

/** 底盘初始化任务 **/
static void chassis_init_task(void) {
    /** 使能关节 **/
    joint_enable();

    chassis.init_flag = true;
}

/** 底盘使能任务 **/
static void chassis_enable_task(void) {

//    /** 更新五连杆参数 **/
//    vmc_calc();
//
//    /** 更新底盘变量 **/
//    chassis_variable_update();

    /** 速度融合 **/
    speed_calc();

    /** 计算驱动轮力矩 **/
    wheel_calc();

    /** 计算关节力矩 **/
    joint_calc();

    /** 倒地自救 **/
    chassis_selfhelp();
}

/** 关节MIT模式-发送力矩任务 **/
static void
MIT_send_torque_task(float joint_LF_torque, float joint_LB_torque, float joint_RF_torque, float joint_RB_torque,
                     float wheel_L_torque, float wheel_R_torque,
                     float L_speed, float L_Kd,
                     float R_speed, float R_Kd) {
    /** joint **/
    set_left_dm8009p_MIT(&joint[LF], 0.0f, L_speed, 0, L_Kd, joint_LF_torque);
    set_left_dm8009p_MIT(&joint[LB], 0.0f, L_speed, 0, L_Kd, joint_LB_torque);

    set_right_dm8009p_MIT(&joint[RF], 0.0f, R_speed, 0, R_Kd, joint_RF_torque);
    set_right_dm8009p_MIT(&joint[RB], 0.0f, R_speed, 0, R_Kd, joint_RB_torque);

    /** wheel **/
    int16_t L_wheel_data = wheel_L_torque * (1 / TORQUE_CONSTANT_3508) * DATA_PER_A;
    int16_t R_wheel_data = wheel_R_torque * (1 / TORQUE_CONSTANT_3508) * DATA_PER_A;

    DJI_Current_Set(L_wheel_data,
                    R_wheel_data,
                    0,
                    0);

}

void chassis_task(void) {

    /** 获取底盘遥控器信息(模式 + 数据) **/
    chassis_remote_cmd();

    /** 获取传感器数据 **/
    get_IMU_info();

    /** 更新五连杆参数 **/
    vmc_calc();

    /** 更新底盘变量 **/
    chassis_variable_update();

    switch (chassis.chassis_ctrl_mode) {
        case CHASSIS_DISABLE: {
            chassis_disable_task();
            break;
        }

        case CHASSIS_INIT: {
            chassis_init_task();
            break;
        }

        case CHASSIS_ENABLE: {
            chassis_enable_task();
            break;
        }

        default: {
            break;
        }
    }

    MIT_send_torque_task(0,
                         0,
                         0,
                         0,
                         0,
                         0,
                         0,
                         0,
                         0,
                         0);

//    MIT_send_torque_task(chassis.leg_L.joint_F_torque,
//                         chassis.leg_L.joint_B_torque,
//                         -chassis.leg_R.joint_F_torque,
//                         -chassis.leg_R.joint_B_torque,
//                         chassis.leg_L.wheel_torque,
//                         chassis.leg_R.wheel_torque,
//                         0,
//                         0,
//                         0,
//                         0);

}
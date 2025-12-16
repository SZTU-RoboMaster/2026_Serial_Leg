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
#include "ins_task.h"
#include "vx_kalman_filter.h"
#include "lqr.h"
#include "bsp_delay.h"
#include "bsp_dwt.h"
#include "vofa.h"

bool joint_enable_flag = false; // 确保初始化任务时关节使能函数只执行一次
bool left_leg_need_reset = false;
bool right_leg_need_reset = false;

bool leg_need_shou_tui = false; // 收腿

bool temp_flag = false;

float L_Kd = 0;
float L_Speed = 0;

float R_Kd = 0;
float R_Speed = 0;

#define THETA_OFFSET (10.0f * DEGREE_TO_RAD)

/** 底盘pid初始化 **/
static void chassis_pid_init() {

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

    // 离地后的腿长PID 暂时没用到
    pid_init(&chassis.leg_L.offground_leg_pid,
             CHASSIS_OFFGROUND_L0_PID_OUT_LIMIT,
             CHASSIS_OFFGROUND_L0_PID_IOUT_LIMIT,
             CHASSIS_OFFGROUND_LO_PID_P,
             CHASSIS_OFFGROUND_L0_PID_I,
             CHASSIS_OFFGROUND_L0_PID_D);

    pid_init(&chassis.leg_R.offground_leg_pid,
             CHASSIS_OFFGROUND_L0_PID_OUT_LIMIT,
             CHASSIS_OFFGROUND_L0_PID_IOUT_LIMIT,
             CHASSIS_OFFGROUND_LO_PID_P,
             CHASSIS_OFFGROUND_L0_PID_I,
             CHASSIS_OFFGROUND_L0_PID_D);

    // Roll补偿PID
    pid_init(&chassis.chassis_roll_pid,
             CHASSIS_ROLL_PID_OUT_LIMIT,
             CHASSIS_ROLL_PID_IOUT_LIMIT,
             CHASSIS_ROLL_PID_P,
             CHASSIS_ROLL_PID_I,
             CHASSIS_ROLL_PID_D);
}

/** 底盘初始化 **/
void chassis_init(void)
{
    /** 初始化底盘模式 **/
    chassis.chassis_ctrl_mode = CHASSIS_DISABLE;

    /** 关节电机初始化 **/
    joint_init();

    /** 轮毂电机初始化 **/
    wheel_init();

    /** 底盘pid初始化 **/
    chassis_pid_init();

    /** 滤波器初始化 **/
    // 低通滤波
    low_pass_filter_init(&chassis.leg_L.theta_dot_lpf, 0.75f);
    low_pass_filter_init(&chassis.leg_R.theta_dot_lpf, 0.75f);

    // 卡尔曼滤波
    xvEstimateKF_Init(&vaEstimateKF);
}

/** 底盘倒地自救 **/
static void chassis_selfhelp(void)
{
    /** 检测是否需要自救 **/
    


    /** 倒地自救尚未完成 **/

}

/** 获取底盘传感器数据 **/
static void get_IMU_info(void) {

    /** Yaw **/
    chassis.imu_reference.yaw_rad = -INS.Yaw * DEGREE_TO_RAD;

    chassis.imu_reference.yaw_total_rad = -INS.YawTotalAngle * DEGREE_TO_RAD;

    /** Pitch **/
    chassis.imu_reference.pitch_rad = -INS.Roll * DEGREE_TO_RAD;

    /** Roll **/
    chassis.imu_reference.roll_rad = INS.Pitch * DEGREE_TO_RAD;

    /** 更新各轴加速度和角速度 **/
    chassis.imu_reference.pitch_gyro = -INS.Gyro[Y];
    chassis.imu_reference.yaw_gyro = -INS.Gyro[Z];
    chassis.imu_reference.roll_gyro = INS.Gyro[X];

    chassis.imu_reference.ax = INS.Accel[X];
    chassis.imu_reference.ay = INS.Accel[Y];
    chassis.imu_reference.az = INS.Accel[Z];

    /** 机体竖直方向加速度 **/
    chassis.imu_reference.robot_az = INS.MotionAccel_n[Z];
}

/** 更新底盘变量 **/
static void chassis_variable_update(void) {

    get_IMU_info();

    // 5.phi
    chassis.leg_L.state_variable_feedback.phi = chassis.imu_reference.pitch_rad;
    chassis.leg_R.state_variable_feedback.phi = chassis.imu_reference.pitch_rad;

    // 6.phi_dot
    chassis.leg_L.state_variable_feedback.phi_dot = chassis.imu_reference.pitch_gyro;
    chassis.leg_R.state_variable_feedback.phi_dot = chassis.imu_reference.pitch_gyro;

    //theta_last
    chassis.leg_L.state_variable_feedback.theta_last = chassis.leg_L.state_variable_feedback.theta;
    chassis.leg_R.state_variable_feedback.theta_last = chassis.leg_R.state_variable_feedback.theta;

    //1.theta
    chassis.leg_L.state_variable_feedback.theta = cal_leg_theta(chassis.leg_L.vmc.forward_kinematics.fk_phi.phi0, chassis.leg_L.state_variable_feedback.phi);
    chassis.leg_R.state_variable_feedback.theta = cal_leg_theta(chassis.leg_R.vmc.forward_kinematics.fk_phi.phi0, chassis.leg_R.state_variable_feedback.phi);

    //2. theta_dot
    float theta_dot_raw_L;
    float theta_dot_raw_R;

    theta_dot_raw_L = (chassis.leg_L.state_variable_feedback.theta - chassis.leg_L.state_variable_feedback.theta_last) / (CHASSIS_PERIOD * 0.001f);

    chassis.leg_L.state_variable_feedback.theta_dot_last = chassis.leg_L.state_variable_feedback.theta_dot;
    chassis.leg_L.state_variable_feedback.theta_dot = update_low_pass_filter(&chassis.leg_L.theta_dot_lpf, theta_dot_raw_L);

    theta_dot_raw_R = (chassis.leg_R.state_variable_feedback.theta - chassis.leg_R.state_variable_feedback.theta_last) / (CHASSIS_PERIOD * 0.001f);

    chassis.leg_R.state_variable_feedback.theta_dot_last = chassis.leg_R.state_variable_feedback.theta_dot;
    chassis.leg_R.state_variable_feedback.theta_dot = update_low_pass_filter(&chassis.leg_R.theta_dot_lpf, theta_dot_raw_R);

    // 2.1 theta_ddot
    chassis.leg_L.state_variable_feedback.theta_ddot = (chassis.leg_L.state_variable_feedback.theta_dot - chassis.leg_L.state_variable_feedback.theta_dot_last) / (CHASSIS_PERIOD * 0.001f);
    chassis.leg_R.state_variable_feedback.theta_ddot = (chassis.leg_R.state_variable_feedback.theta_dot - chassis.leg_R.state_variable_feedback.theta_dot_last) / (CHASSIS_PERIOD * 0.001f);


    //4.x_dot
    chassis.leg_L.state_variable_feedback.x_dot = vel_acc[0];
    chassis.leg_R.state_variable_feedback.x_dot = vel_acc[0];

    //3.x
    if(chassis.chassis_ctrl_info.v_m_per_s != 0.0f)
    {
        chassis.leg_L.state_variable_feedback.x = 0.0f;
        chassis.leg_R.state_variable_feedback.x = 0.0f;
    }
    else
    {
        chassis.leg_L.state_variable_feedback.x += (CHASSIS_PERIOD * 0.001f) * chassis.leg_L.state_variable_feedback.x_dot;
        chassis.leg_R.state_variable_feedback.x += (CHASSIS_PERIOD * 0.001f) * chassis.leg_R.state_variable_feedback.x_dot;

//        if((ABS(chassis.leg_L.state_variable_feedback.x_dot) < 0.5f) && (ABS(chassis.leg_R.state_variable_feedback.x_dot) < 0.5f))
//        {
//            chassis.leg_L.state_variable_feedback.x += (CHASSIS_PERIOD * 0.001f) * chassis.leg_L.state_variable_feedback.x_dot;
//            chassis.leg_R.state_variable_feedback.x += (CHASSIS_PERIOD * 0.001f) * chassis.leg_R.state_variable_feedback.x_dot;
//        }
//        else
//        {
//            chassis.leg_L.state_variable_feedback.x = 0.0f;
//            chassis.leg_R.state_variable_feedback.x = 0.0f;
//        }
    }

    // 4.1 x_ddot
    chassis.leg_L.state_variable_feedback.x_dot_last = chassis.leg_L.state_variable_feedback.x_dot;
    chassis.leg_L.state_variable_feedback.x_ddot = (chassis.leg_L.state_variable_feedback.x_dot - chassis.leg_L.state_variable_feedback.x_dot_last) / (CHASSIS_PERIOD * 0.001f);

    chassis.leg_R.state_variable_feedback.x_dot_last = chassis.leg_R.state_variable_feedback.x_dot;
    chassis.leg_R.state_variable_feedback.x_ddot = (chassis.leg_R.state_variable_feedback.x_dot - chassis.leg_R.state_variable_feedback.x_dot_last) / (CHASSIS_PERIOD * 0.001f);
}

/** 计算驱动轮力矩 **/
static void wheel_calc(void)
{
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


      /** 板凳模型 **/
    chassis.leg_L.wheel_torque = - wheel_K_L[0] * (chassis.leg_L.state_variable_feedback.theta - THETA_OFFSET)
                                 - wheel_K_L[1] * (chassis.leg_L.state_variable_feedback.theta_dot - 0.0f)
                                 - wheel_K_L[2] * (chassis.leg_L.state_variable_feedback.x - 0.0f)
                                 - wheel_K_L[3] * (chassis.leg_L.state_variable_feedback.x_dot - chassis.chassis_ctrl_info.v_m_per_s)
                                 - wheel_K_L[4] * (chassis.leg_L.state_variable_feedback.phi - 0.0f)
                                 - wheel_K_L[5] * (chassis.leg_L.state_variable_feedback.phi_dot - 0.0f);

    chassis.leg_R.wheel_torque = - wheel_K_R[0] * (chassis.leg_R.state_variable_feedback.theta - THETA_OFFSET)
                                 - wheel_K_R[1] * (chassis.leg_R.state_variable_feedback.theta_dot - 0.0f)
                                 - wheel_K_R[2] * (chassis.leg_R.state_variable_feedback.x - 0.0f)
                                 - wheel_K_R[3] * (chassis.leg_R.state_variable_feedback.x_dot - chassis.chassis_ctrl_info.v_m_per_s)
                                 - wheel_K_R[4] * (chassis.leg_R.state_variable_feedback.phi - 0.0f)
                                 - wheel_K_R[5] * (chassis.leg_R.state_variable_feedback.phi_dot - 0.0f);

    /*************/

    chassis.leg_L.wheel_torque -= chassis.wheel_turn_torque;
    chassis.leg_R.wheel_torque += chassis.wheel_turn_torque;

    chassis.leg_L.wheel_torque *= -1;

    VAL_LIMIT(chassis.leg_L.wheel_torque, MIN_WHEEL_TORQUE, MAX_WHEEL_TORQUE);
    VAL_LIMIT(chassis.leg_R.wheel_torque, MIN_WHEEL_TORQUE, MAX_WHEEL_TORQUE);

}

/** 计算关节力矩 **/
static void joint_calc(void)
{
/******************************* Joint *************************************/

    chassis_K_matrix_fitting(chassis.leg_L.vmc.forward_kinematics.fk_L0.L0, joint_K_L, joint_fitting_factor);
    chassis_K_matrix_fitting(chassis.leg_R.vmc.forward_kinematics.fk_L0.L0, joint_K_R, joint_fitting_factor);

    /** Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp **/

    /****** 防劈叉pid ******/
    chassis.steer_compensatory_torque =  CHASSIS_LEG_COORDINATION_PID_P * (0.0f - chassis.phi0_error)
                                         + CHASSIS_LEG_COORDINATION_PID_D * (0.0f - chassis.d_phi0_error); // 注意微分项正负

    if(chassis.chassis_ctrl_mode == CHASSIS_INIT)
    {
        if(!leg_need_shou_tui)
        {
            chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point =  0.0f;
            chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point =  0.0f;
        }
        else
        {
            chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point =  - joint_K_L[0] * (chassis.leg_L.state_variable_feedback.theta - 0.0f)
                                                                                 - joint_K_L[1] * (chassis.leg_L.state_variable_feedback.theta_dot - 0.0f);


            chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point =  - joint_K_R[0] * (chassis.leg_R.state_variable_feedback.theta - 0.0f)
                                                                                 - joint_K_R[1] * (chassis.leg_R.state_variable_feedback.theta_dot - 0.0f);

            chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis.steer_compensatory_torque;
            chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point -= chassis.steer_compensatory_torque;
        }
    }
    else
    {
//        chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point =  - joint_K_L[0] * (chassis.leg_L.state_variable_feedback.theta - THETA_OFFSET)
//                                                                             - joint_K_L[1] * (chassis.leg_L.state_variable_feedback.theta_dot - 0.0f)
//                                                                             - joint_K_L[2] * (chassis.leg_L.state_variable_feedback.x - 0.0f)
//                                                                             - joint_K_L[3] * (chassis.leg_L.state_variable_feedback.x_dot - chassis.chassis_ctrl_info.v_m_per_s)
//                                                                             - joint_K_L[4] * (chassis.leg_L.state_variable_feedback.phi - 0.0f)
//                                                                             - joint_K_L[5] * (chassis.leg_L.state_variable_feedback.phi_dot - 0.0f);
//
//
//        chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point =  - joint_K_R[0] * (chassis.leg_R.state_variable_feedback.theta - THETA_OFFSET)
//                                                                             - joint_K_R[1] * (chassis.leg_R.state_variable_feedback.theta_dot - 0.0f)
//                                                                             - joint_K_R[2] * (chassis.leg_R.state_variable_feedback.x - 0.0f)
//                                                                             - joint_K_R[3] * (chassis.leg_R.state_variable_feedback.x_dot - chassis.chassis_ctrl_info.v_m_per_s)
//                                                                             - joint_K_R[4] * (chassis.leg_R.state_variable_feedback.phi - 0.0f)
//                                                                             - joint_K_R[5] * (chassis.leg_R.state_variable_feedback.phi_dot - 0.0f);

        chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point =  - joint_K_L[0] * (chassis.leg_L.state_variable_feedback.theta - THETA_OFFSET)
                                                                             - joint_K_L[1] * (chassis.leg_L.state_variable_feedback.theta_dot - 0.0f);


        chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point =  - joint_K_R[0] * (chassis.leg_R.state_variable_feedback.theta - THETA_OFFSET)
                                                                             - joint_K_R[1] * (chassis.leg_R.state_variable_feedback.theta_dot - 0.0f);

    chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis.steer_compensatory_torque;
    chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point -= chassis.steer_compensatory_torque;

        /** 不要在Tp层面修改左右侧旋转方向相反这个问题 因为解算出来加到电机上的力矩是对的 **/
    }

    /** End End End End End End End End End End End End End End End End End End End End End End End End **/


    /** F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F **/

    /****** Leg pid ******/
    if(chassis.chassis_ctrl_mode == CHASSIS_INIT)
    {
        if(leg_need_shou_tui)
        {
            chassis.chassis_ctrl_info.target_length = MIN_L0;
        }
    }
    else if(chassis.chassis_ctrl_mode == CHASSIS_ENABLE)
    {
        chassis.chassis_ctrl_info.target_length = MID_L0;
    }

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

    chassis.roll_compensatory_torque =  CHASSIS_ROLL_PID_P * (0.0f - chassis.imu_reference.roll_rad)
                                      + CHASSIS_ROLL_PID_D * (0.0f - chassis.imu_reference.roll_gyro); // 注意微分项正负

    if(chassis.chassis_ctrl_mode == CHASSIS_INIT)// 初始化模式
    {
        if(left_leg_need_reset)
        {
            // 确保腿复位时伸直
            chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point =  0.5f * chassis_physical_config.body_weight * GRAVITY;
        }
        else
        {
            chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point = 0;
        }

        if(right_leg_need_reset)
        {
            chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point =  0.5f * chassis_physical_config.body_weight * GRAVITY;
        }
        else
        {
            chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point = 0;
        }

        if(leg_need_shou_tui)
        {
            chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point =  chassis.leg_L.leg_speed_pid.out;
            chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point =  chassis.leg_R.leg_speed_pid.out;
        }
    }
    else // 使能模式
    {
        chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point =  0.5f * chassis_physical_config.body_weight * GRAVITY * cosf(chassis.leg_L.state_variable_feedback.theta)
                                                                             + chassis.leg_L.leg_speed_pid.out;

        chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point =  0.5f * chassis_physical_config.body_weight * GRAVITY * cosf(chassis.leg_R.state_variable_feedback.theta)
                                                                             + chassis.leg_R.leg_speed_pid.out;

        chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point += chassis.roll_compensatory_torque;
        chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point -= chassis.roll_compensatory_torque;
    }



//    chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point = 0.0f;
//    chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point = 0.0f;

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
static void chassis_disable_task() {

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
    joint_enable_flag = false;

    chassis.chassis_recover_finish = false;

    left_leg_need_reset = false;
    right_leg_need_reset = false;
    leg_need_shou_tui = false;

    L_Kd = 0;
    L_Speed = 0;
    R_Kd = 0;
    R_Speed = 0;

    temp_flag = false;
}

/** 底盘初始化任务 **/
static void chassis_init_task()
{
    /** 使能关节 **/
    if(!joint_enable_flag)
    {
        joint_enable();

        joint_enable_flag = true;
    }

    /** 更新五连杆参数 **/
    vmc_calc();

    /** 更新底盘变量 **/
    chassis_variable_update();

    /** 驱动轮 **/
    chassis.leg_L.wheel_torque = 0.0f;
    chassis.leg_R.wheel_torque = 0.0f;

    /** 关节 **/
    if(chassis.leg_L.state_variable_feedback.theta < 0.0f)
    {
        left_leg_need_reset = true;
        L_Kd = 3.0f;
        L_Speed = -1.5f;
    }
    else
    {
        left_leg_need_reset = false;
        L_Kd = 0.0f;
        L_Speed = 0.0f;
    }

    if(chassis.leg_R.state_variable_feedback.theta < 0.0f)
    {
        right_leg_need_reset = true;
        R_Kd = 3.0f;
        R_Speed = 1.5f;
    }
    else
    {
        right_leg_need_reset = false;
        R_Kd = 0.0f;
        R_Speed = 0.0f;
    }

    joint_calc();

    /***********/

    if((!left_leg_need_reset) && (!right_leg_need_reset)) {

        if ((chassis.leg_L.state_variable_feedback.theta < 25.0f * DEGREE_TO_RAD)
            && (chassis.leg_R.state_variable_feedback.theta < 25.0f * DEGREE_TO_RAD)) {

            temp_flag = true;
        }

        if(temp_flag)
        {
            leg_need_shou_tui = false;

            chassis.init_flag = true;
        }
        else {
            leg_need_shou_tui = true;

            chassis.init_flag = false;
        }

    }
    else
    {
        chassis.init_flag = false;
    }

//    chassis.init_flag = true;
}

/** 底盘使能任务 **/
static void chassis_enable_task(void)
{
    /** 更新五连杆参数 **/
    vmc_calc();

    /** 速度融合 **/
    speed_calc();

    /** 更新底盘变量 **/
    chassis_variable_update();

    /** 计算驱动轮力矩 **/
    wheel_calc();

    /** 计算关节力矩 **/
    joint_calc();

    /** 倒地自救 **/
    chassis_selfhelp();
}

/** 发送力矩任务 **/
static void send_torque_task(float joint_LF_torque, float joint_LB_torque, float joint_RF_torque, float joint_RB_torque,
                             float wheel_L_torque, float wheel_R_torque,
                             float L_speed, float L_Kd,
                             float R_speed, float R_Kd)
{
    /** joint **/
    set_dm8009p_MIT(&joint[LF],0.0f, L_speed, 0, L_Kd,joint_LF_torque);
    set_dm8009p_MIT(&joint[LB],0.0f, L_speed, 0, L_Kd,joint_LB_torque);
    DWT_Delay(0.0002f);
    set_dm8009p_MIT(&joint[RF],0.0f, R_speed, 0, R_Kd,joint_RF_torque);
    set_dm8009p_MIT(&joint[RB],0.0f, R_speed, 0, R_Kd,joint_RB_torque);

    /** wheel **/
    int16_t L_wheel_data = wheel_L_torque * (1 / TORQUE_CONSTANT_3508) * DATA_PER_A;
    int16_t R_wheel_data = wheel_R_torque * (1 / TORQUE_CONSTANT_3508) * DATA_PER_A;

    DJI_Current_Set(L_wheel_data,
                    R_wheel_data,
                    0,
                    0);

}


void chassis_task(void)
{
    /** 获取遥控器信息(模式 + 数据) **/
    remote_cmd();

    switch (chassis.chassis_ctrl_mode)
    {
        case CHASSIS_DISABLE:
        {
            chassis_disable_task();
            break;
        }


        case CHASSIS_INIT:
        {
            chassis_init_task();
            break;
        }


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

//    send_torque_task(chassis.leg_L.joint_F_torque,
//                     chassis.leg_L.joint_B_torque,
//                     -chassis.leg_R.joint_F_torque,
//                     -chassis.leg_R.joint_B_torque,
//                     0.0f,
//                     0.0f,
//                     0.0f,
//                     0.0f,
//                     0.0f,
//                     0.0f);

//    send_torque_task(chassis.leg_L.joint_F_torque,
//                     chassis.leg_L.joint_B_torque,
//                     -chassis.leg_R.joint_F_torque,
//                     -chassis.leg_R.joint_B_torque,
//                     0.0f,
//                     0.0f,
//                     L_Speed,
//                     L_Kd,
//                     R_Speed,
//                     R_Kd);

    send_torque_task(chassis.leg_L.joint_F_torque,
                     chassis.leg_L.joint_B_torque,
                     -chassis.leg_R.joint_F_torque,
                     -chassis.leg_R.joint_B_torque,
                     chassis.leg_L.wheel_torque,
                     chassis.leg_R.wheel_torque,
                     L_Speed,
                     L_Kd,
                     R_Speed,
                     R_Kd);

//    send_torque_task(0,
//                     0,
//                     0,
//                     0,
//                     -chassis.leg_L.wheel_torque,
//                     chassis.leg_R.wheel_torque,
//                     L_Speed,
//                     L_Kd,
//                     R_Speed,
//                     R_Kd);

//    send_torque_task(chassis.leg_L.joint_F_torque,
//                     chassis.leg_L.joint_B_torque,
//                     0.0f,
//                     0.0f,
//                     0.0f,
//                     0.0f,
//                     0.0f,
//                     0.0f);

//    send_torque_task(0.0f,
//                     0.0f,
//                     -chassis.leg_R.joint_F_torque,
//                     -chassis.leg_R.joint_B_torque,
//                     0.0f,
//                     0.0f,
//                     0.0f,
//                     0.0f);

//    send_torque_task(chassis.leg_L.joint_F_torque,
//                     chassis.leg_L.joint_B_torque,
//                     -chassis.leg_R.joint_F_torque,
//                     -chassis.leg_R.joint_B_torque,
//                     -chassis.leg_L.wheel_torque,
//                     chassis.leg_R.wheel_torque,
//                     0.0f,
//                     0.0f,
//                     0.0f,
//                     0.0f);

//    send_torque_task(0.0f,
//                     0.0f,
//                     0.0f,
//                     0.0f,
//                     0.0f,
//                     0.0f,
//                     L_Speed,
//                     L_Kd,
//                     R_Speed,
//                     R_Kd);

//    send_torque_task(0.0f,
//                     0.0f,
//                     0.0f,
//                     0.0f,
//                     0.0f,
//                     0.0f,
//                     0,
//                     0,
//                     0,
//                     0);


}
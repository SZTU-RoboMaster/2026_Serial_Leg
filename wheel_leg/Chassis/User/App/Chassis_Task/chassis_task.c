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

/** 底盘自起状态判断 **/
static void chassis_recover_state_check(void) {

    if(chassis.chassis_recover_finish == false)
    {
        // 机体姿态检测
        if (ABS(chassis.imu_reference.pitch_rad) <= NOT_BALANCE_RAD) {
            chassis.chassis_body_state = CHASSIS_BODY_NORMAL;
        } else {
            chassis.chassis_body_state = CHASSIS_BODY_UNNORMAL;
        }

        // 判断倒地时的腿倾角姿态
        if((ABS(chassis.leg_L.state_variable_feedback.theta) <= LEG_NORMAL_RAD)
           && (ABS(chassis.leg_R.state_variable_feedback.theta) <= LEG_NORMAL_RAD))
        {
            chassis.chassis_fall_leg_state = CHASSIS_FALL_LEG_NORMAL;
        }
        else
        {
            chassis.chassis_fall_leg_state = CHASSIS_FALL_LEG_UNNORMAL;
        }

        // 倒地自起判断
        // 倒地时，只有当腿倾角处于正常范围才允许自起
        if(chassis.chassis_fall_leg_state == CHASSIS_FALL_LEG_NORMAL)
        {
            chassis.chassis_recover_state = CHASSIS_COULD_RECOVER;
        }
        else
        {
            chassis.chassis_recover_state = CHASSIS_COULD_NOT_RECOVER;
        }

    }
}


/** 底盘倒地自救 **/
static void chassis_selfhelp(void)
{
    chassis_recover_state_check();

    if(chassis.chassis_recover_finish == false)
    {
        chassis.leg_L.joint_B_torque = 0.0f;
        chassis.leg_L.joint_F_torque = 0.0f;
        chassis.leg_R.joint_B_torque = 0.0f;
        chassis.leg_R.joint_F_torque = 0.0f;

        if(chassis.chassis_recover_state == CHASSIS_COULD_RECOVER)
        {
            Kd = 0.0f;
            vel = 0.0f;
        }
        else
        {
            chassis.leg_L.wheel_torque = 0;
            chassis.leg_R.wheel_torque = 0;

            // 腿复位
            if(chassis.imu_reference.pitch_rad < 0.0f)
            {
                Kd = 3.0f;
                vel = -2.0f;
            }
            else
            {
                Kd = 3.0f;
                vel = 2.0f;
            }

        }
    }

    if(ABS(chassis.imu_reference.pitch_rad) <= RECOVER_RAD)
    {
        chassis.chassis_recover_finish = true;
    }
    else if(ABS(chassis.imu_reference.pitch_rad) >= NOT_BALANCE_RAD)
    {
        chassis.chassis_recover_finish = false;
    }
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
        chassis.leg_L.state_variable_feedback.x = chassis.leg_L.state_variable_feedback.x + CHASSIS_PERIOD * 0.001f * chassis.leg_L.state_variable_feedback.x_dot;
        chassis.leg_R.state_variable_feedback.x = chassis.leg_R.state_variable_feedback.x + CHASSIS_PERIOD * 0.001f * chassis.leg_R.state_variable_feedback.x_dot;

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
//    chassis_K_matrix_fitting(chassis.leg_L.vmc.forward_kinematics.fk_L0.L0, wheel_K_L, wheel_fitting_factor);
//    chassis_K_matrix_fitting(chassis.leg_R.vmc.forward_kinematics.fk_L0.L0, wheel_K_R, wheel_fitting_factor);

    float target_yaw_speed = pid_calc(&chassis.chassis_turn_pos_pid,
                                      chassis.imu_reference.yaw_total_rad,
                                      chassis.chassis_ctrl_info.yaw_rad);

    chassis.wheel_turn_torque = -pid_calc(&chassis.chassis_turn_speed_pid,
                                         chassis.imu_reference.yaw_gyro,
                                         target_yaw_speed);


    /** theta √ **/
//    chassis.leg_L.wheel_torque = - wheel_K_L[0] * (chassis.leg_L.state_variable_feedback.theta - 0.0f)
//                                 - wheel_K_L[1] * (chassis.leg_L.state_variable_feedback.theta_dot - 0.0f);
//
//    chassis.leg_R.wheel_torque = - wheel_K_R[0] * (chassis.leg_R.state_variable_feedback.theta - 0.0f)
//                                 - wheel_K_R[1] * (chassis.leg_R.state_variable_feedback.theta_dot - 0.0f);

    /** x √ **/
//    chassis.leg_L.wheel_torque =   wheel_K_L[2] * (chassis.leg_L.state_variable_feedback.x - 0.0f)
//                                 + wheel_K_L[3] * (chassis.leg_L.state_variable_feedback.x_dot - chassis.chassis_ctrl_info.v_m_per_s);
//
//    chassis.leg_R.wheel_torque =   wheel_K_R[2] * (chassis.leg_R.state_variable_feedback.x - 0.0f)
//                                 + wheel_K_R[3] * (chassis.leg_R.state_variable_feedback.x_dot - chassis.chassis_ctrl_info.v_m_per_s);

    /** pitch √ **/
//    chassis.leg_L.wheel_torque = - wheel_K[4] * (chassis.leg_L.state_variable_feedback.phi - 0.0f)
//                                 - wheel_K[5] * (chassis.leg_L.state_variable_feedback.phi_dot - 0.0f);
//
//    chassis.leg_R.wheel_torque = - wheel_K[4] * (chassis.leg_R.state_variable_feedback.phi - 0.0f)
//                                 - wheel_K[5] * (chassis.leg_R.state_variable_feedback.phi_dot - 0.0f);

    /** All **/
//    chassis.leg_L.wheel_torque = - wheel_K[0] * (chassis.leg_L.state_variable_feedback.theta - 0.0f)
//                                 - wheel_K[1] * (chassis.leg_L.state_variable_feedback.theta_dot - 0.0f)
//                                 - wheel_K[4] * (chassis.leg_L.state_variable_feedback.phi - 0.0f)
//                                 - wheel_K[5] * (chassis.leg_L.state_variable_feedback.phi_dot - 0.0f);
//
//    chassis.leg_R.wheel_torque = - wheel_K[0] * (chassis.leg_R.state_variable_feedback.theta - 0.0f)
//                                 - wheel_K[1] * (chassis.leg_R.state_variable_feedback.theta_dot - 0.0f)
//                                 - wheel_K[4] * (chassis.leg_R.state_variable_feedback.phi - 0.0f)
//                                 - wheel_K[5] * (chassis.leg_R.state_variable_feedback.phi_dot - 0.0f);

//    chassis.leg_L.wheel_torque =   -wheel_K[2] * (chassis.leg_L.state_variable_feedback.x - 0.0f)
//                                 - wheel_K[3] * (chassis.leg_L.state_variable_feedback.x_dot - chassis.chassis_ctrl_info.v_m_per_s)
//                                 - wheel_K[4] * (chassis.leg_L.state_variable_feedback.phi - 0.0f)
//                                 - wheel_K[5] * (chassis.leg_L.state_variable_feedback.phi_dot - 0.0f);
//
//    chassis.leg_R.wheel_torque =   -wheel_K[2] * (chassis.leg_R.state_variable_feedback.x - 0.0f)
//                                 - wheel_K[3] * (chassis.leg_R.state_variable_feedback.x_dot - chassis.chassis_ctrl_info.v_m_per_s)
//                                 - wheel_K[4] * (chassis.leg_R.state_variable_feedback.phi - 0.0f)
//                                 - wheel_K[5] * (chassis.leg_R.state_variable_feedback.phi_dot - 0.0f);

      /** 尝试板凳模型平衡 **/
    chassis.leg_L.wheel_torque =   - wheel_K[2] * (chassis.leg_L.state_variable_feedback.x - 0.0f)
                                   - wheel_K[3] * (chassis.leg_L.state_variable_feedback.x_dot - chassis.chassis_ctrl_info.v_m_per_s)
                                   - wheel_K[4] * (chassis.leg_L.state_variable_feedback.phi - 0.0f)
                                   - wheel_K[5] * (chassis.leg_L.state_variable_feedback.phi_dot - 0.0f);

    chassis.leg_R.wheel_torque =   - wheel_K[2] * (chassis.leg_R.state_variable_feedback.x - 0.0f)
                                   - wheel_K[3] * (chassis.leg_R.state_variable_feedback.x_dot - chassis.chassis_ctrl_info.v_m_per_s)
                                   - wheel_K[4] * (chassis.leg_R.state_variable_feedback.phi - 0.0f)
                                   - wheel_K[5] * (chassis.leg_R.state_variable_feedback.phi_dot - 0.0f);
       /*******************/

       /** 前进后退方向正确 **/
//    chassis.leg_L.wheel_torque =     wheel_K[2] * (chassis.leg_L.state_variable_feedback.x - 0.0f)
//                                     + wheel_K[3] * (chassis.leg_L.state_variable_feedback.x_dot - chassis.chassis_ctrl_info.v_m_per_s);
//
//    chassis.leg_R.wheel_torque =     wheel_K[2] * (chassis.leg_R.state_variable_feedback.x - 0.0f)
//                                     + wheel_K[3] * (chassis.leg_R.state_variable_feedback.x_dot - chassis.chassis_ctrl_info.v_m_per_s);
       /*******************/

//    chassis.leg_L.wheel_torque -= chassis.wheel_turn_torque;
//    chassis.leg_R.wheel_torque += chassis.wheel_turn_torque;
    chassis.leg_L.wheel_torque *= -1;

    VAL_LIMIT(chassis.leg_L.wheel_torque, MIN_WHEEL_TORQUE, MAX_WHEEL_TORQUE);
    VAL_LIMIT(chassis.leg_R.wheel_torque, MIN_WHEEL_TORQUE, MAX_WHEEL_TORQUE);

}

/** 计算关节力矩 **/

static void joint_calc(void)
{
/******************************* Joint *************************************/

//    chassis_K_matrix_fitting(chassis.leg_L.vmc.forward_kinematics.fk_L0.L0, joint_K_L, joint_fitting_factor);
//    chassis_K_matrix_fitting(chassis.leg_R.vmc.forward_kinematics.fk_L0.L0, joint_K_R, joint_fitting_factor);

    /** Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp **/



    /** End End End End End End End End End End End End End End End End End End End End End End End End **/


    /** F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F **/



    /** End End End End End End End End End End End End End End End End End End End End End End End End **/

    // 计算关节电机力矩
    vmc_forward_dynamics(&chassis.leg_L.vmc, &chassis_physical_config);
    vmc_forward_dynamics(&chassis.leg_R.vmc, &chassis_physical_config);

    chassis.leg_L.joint_F_torque = chassis.leg_L.vmc.forward_kinematics.T1_T4_set_point.E.T1_set_point;//F
    chassis.leg_L.joint_B_torque = chassis.leg_L.vmc.forward_kinematics.T1_T4_set_point.E.T4_set_point;//B

    chassis.leg_R.joint_F_torque = chassis.leg_R.vmc.forward_kinematics.T1_T4_set_point.E.T1_set_point;//F
    chassis.leg_R.joint_B_torque = chassis.leg_R.vmc.forward_kinematics.T1_T4_set_point.E.T4_set_point;//B

    // 输出限幅
    VAL_LIMIT(chassis.leg_R.joint_F_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
    VAL_LIMIT(chassis.leg_R.joint_B_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
    VAL_LIMIT(chassis.leg_L.joint_F_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
    VAL_LIMIT(chassis.leg_L.joint_B_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);

}

/** 控制器计算 **/
static void controller_calc(void)
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

    chassis.chassis_ctrl_info.height_m = MIN_L0;

    // 底盘状态
    chassis.chassis_body_state = CHASSIS_BODY_UNNORMAL;

    chassis.chassis_fall_leg_state = CHASSIS_FALL_LEG_UNNORMAL;

    chassis.chassis_recover_state = CHASSIS_COULD_NOT_RECOVER;

    /** 初始化标志位 **/

    // 底盘初始化标志位
    chassis.init_flag = false;

    chassis.chassis_recover_finish = false;

}

/** 底盘初始化任务 **/
static void chassis_init_task()
{
    joint_enable();

    chassis.init_flag = true;
}

/** 底盘使能任务 **/
static void chassis_enable_task(void)
{
    /** 控制器计算 **/
    controller_calc();
}

/** 发送力矩任务 **/
static void send_torque_task(float joint_LF_torque, float joint_LB_torque, float joint_RF_torque, float joint_RB_torque,
                             float wheel_L_torque, float wheel_R_torque,
                             float vel, float Kd)
{
    set_dm8009p_MIT(&joint[LF],0.0f, vel, 0, Kd,joint_LF_torque);
    set_dm8009p_MIT(&joint[LB],0.0f, vel, 0, Kd,joint_LB_torque);
    DWT_Delay(0.0002f);
    set_dm8009p_MIT(&joint[RF],0.0f, -vel, 0, Kd,joint_RF_torque);
    set_dm8009p_MIT(&joint[RB],0.0f, -vel, 0, Kd,joint_RB_torque);

    DJI_Current_Set(wheel_L_torque * (1 / TORQUE_CONSTANT_3508) * DATA_PER_A,
                    wheel_R_torque * (1 / TORQUE_CONSTANT_3508) * DATA_PER_A,
                    0,
                    0);

}

static void temp_send_wheel_torque(float wheel_L_torque, float wheel_R_torque)
{
    int16_t L_wheel_data = wheel_L_torque * (1 / TORQUE_CONSTANT_3508) * DATA_PER_A;
    int16_t R_wheel_data = wheel_R_torque * (1 / TORQUE_CONSTANT_3508) * DATA_PER_A;

    DJI_Current_Set(L_wheel_data,
                    R_wheel_data,
                    0,
                    0);
}

static void send_pos_speed_task(float LF_pos, float LB_pos, float RF_pos, float RB_pos,
                                float LF_speed, float LB_speed, float RF_speed, float RB_speed)
{
    set_dm8009p_pos_speed(&joint[LF],LF_pos, LF_speed);
    set_dm8009p_pos_speed(&joint[LB],LB_pos, LB_speed);
    DWT_Delay(0.0002f);
    set_dm8009p_pos_speed(&joint[RF],RF_pos, RF_speed);
    set_dm8009p_pos_speed(&joint[RB],RB_pos, RB_speed);
}

// 腿长0.22  LF_pos：2.74 LB_pos：0.60 RF_pos：-2.74 RB_pos：-0.60

float speed = 0.0f;
float lf_pos = 0.0f;
float lb_pos = 0.0f;
float rf_pos = 0.0f;
float rb_pos = 0.0f;

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

    if (switch_is_mid(rc_ctrl.rc.s[RC_s_R]))
    {
        if(chassis.chassis_ctrl_mode == CHASSIS_ENABLE)
        {
//            speed = 0.0f;
//            lf_pos = 0.0f;
//            lb_pos = 0.0f;
//            rf_pos = 0.0f;
//            rb_pos = 0.0f;

            speed = 0.3f;
            lf_pos = 2.66f;
            lb_pos = 0.58f;
            rf_pos = -2.64f;
            rb_pos = -0.61f;
        }
    }
    else
    {
        speed = 0.0f;
        lf_pos = 0.0f;
        lb_pos = 0.0f;
        rf_pos = 0.0f;
        rb_pos = 0.0f;
    }

    send_pos_speed_task(lf_pos,
                        lb_pos,
                        rf_pos,
                        rb_pos,
                        speed,
                        -speed,
                        -speed,
                        speed);

//    temp_send_wheel_torque(0,0);

    USART_Vofa_Justfloat_Transmit(chassis.leg_L.wheel_torque,chassis.leg_R.wheel_torque,0);

    temp_send_wheel_torque(chassis.leg_L.wheel_torque,chassis.leg_R.wheel_torque);

}
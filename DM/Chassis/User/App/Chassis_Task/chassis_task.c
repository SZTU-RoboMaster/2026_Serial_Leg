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
#include "lqr.h"
#include "bsp_dwt.h"
#include "vofa.h"
#include "dm_imu.h"
#include "ins_task.h"

static uint32_t chassis_dwt_cnt = 0;
static float chassis_dt = CHASSIS_PERIOD * 0.001f;

// Self-help completion debounce.
static uint16_t chassis_selfhelp_exit_count = 0;

// Stand-up hold timer.
static uint16_t chassis_standup_tick = 0;

#define CHASSIS_FALL_PHI0_ENTER           (140.0f * DEGREE_TO_RAD)
#define CHASSIS_SELFHELP_PHI0_TARGET      (0.5f * PI)
#define CHASSIS_SELFHELP_PHI0_EXIT        (5.0f * DEGREE_TO_RAD)
#define CHASSIS_SELFHELP_L0_HOLD_EXIT     0.02f
#define CHASSIS_BALANCE_PITCH_EXIT        (10.0f * DEGREE_TO_RAD)

static float Theta_format(float Angle);

#define CHASSIS_CLEAR_PID_STATE(pid) do { \
    (pid).set = 0.0f; \
    (pid).get = 0.0f; \
    (pid).err[0] = 0.0f; \
    (pid).err[1] = 0.0f; \
    (pid).pout = 0.0f; \
    (pid).iout = 0.0f; \
    (pid).dout = 0.0f; \
    (pid).out = 0.0f; \
    (pid).excess = 0.0f; \
} while (0)

static void chassis_selfhelp_reset_pid_memory(void)
{
    CHASSIS_CLEAR_PID_STATE(chassis.chassis_turn_pos_pid);
    CHASSIS_CLEAR_PID_STATE(chassis.chassis_turn_speed_pid);
    CHASSIS_CLEAR_PID_STATE(chassis.chassis_wheel_x_pos_pid);
    CHASSIS_CLEAR_PID_STATE(chassis.chassis_wheel_x_speed_pid);
    CHASSIS_CLEAR_PID_STATE(chassis.chassis_joint_x_pos_pid);
    CHASSIS_CLEAR_PID_STATE(chassis.chassis_joint_x_speed_pid);
    CHASSIS_CLEAR_PID_STATE(chassis.chassis_leg_coordination_pid);
    CHASSIS_CLEAR_PID_STATE(chassis.leg_L.leg_pos_pid);
    CHASSIS_CLEAR_PID_STATE(chassis.leg_R.leg_pos_pid);
    CHASSIS_CLEAR_PID_STATE(chassis.leg_L.leg_speed_pid);
    CHASSIS_CLEAR_PID_STATE(chassis.leg_R.leg_speed_pid);
    CHASSIS_CLEAR_PID_STATE(chassis.chassis_roll_pid);
    CHASSIS_CLEAR_PID_STATE(chassis.chassis_selfhelp_phi0_pid_L);
    CHASSIS_CLEAR_PID_STATE(chassis.chassis_selfhelp_phi0_pid_R);
}
/** 底盘PID参数初始化 **/
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

    pid_init(&chassis.chassis_wheel_x_pos_pid,
             CHASSIS_WHEEL_X_POS_PID_OUT_LIMIT,
             CHASSIS_WHEEL_X_POS_PID_IOUT_LIMIT,
             CHASSIS_WHEEL_X_POS_PID_P,
             CHASSIS_WHEEL_X_POS_PID_I,
             CHASSIS_WHEEL_X_POS_PID_D);

    pid_init(&chassis.chassis_wheel_x_speed_pid,
             CHASSIS_WHEEL_X_SPEED_PID_OUT_LIMIT,
             CHASSIS_WHEEL_X_SPEED_PID_IOUT_LIMIT,
             CHASSIS_WHEEL_X_SPEED_PID_P,
             CHASSIS_WHEEL_X_SPEED_PID_I,
             CHASSIS_WHEEL_X_SPEED_PID_D);

    pid_init(&chassis.chassis_joint_x_pos_pid,
             CHASSIS_JOINT_X_POS_PID_OUT_LIMIT,
             CHASSIS_JOINT_X_POS_PID_IOUT_LIMIT,
             CHASSIS_JOINT_X_POS_PID_P,
             CHASSIS_JOINT_X_POS_PID_I,
             CHASSIS_JOINT_X_POS_PID_D);

    pid_init(&chassis.chassis_joint_x_speed_pid,
             CHASSIS_JOINT_X_SPEED_PID_OUT_LIMIT,
             CHASSIS_JOINT_X_SPEED_PID_IOUT_LIMIT,
             CHASSIS_JOINT_X_SPEED_PID_P,
             CHASSIS_JOINT_X_SPEED_PID_I,
             CHASSIS_JOINT_X_SPEED_PID_D);

    /** Joint **/
    // 双腿协调PID
    pid_init(&chassis.chassis_leg_coordination_pid,
             CHASSIS_LEG_COORDINATION_PID_OUT_LIMIT,
             CHASSIS_LEG_COORDINATION_PID_IOUT_LIMIT,
             CHASSIS_LEG_COORDINATION_PID_P,
             CHASSIS_LEG_COORDINATION_PID_I,
             CHASSIS_LEG_COORDINATION_PID_D);

    // 腿长位置PID
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

    // 腿长速度PID
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

    // Roll角度PID
    pid_init(&chassis.chassis_roll_pid,
             CHASSIS_ROLL_PID_OUT_LIMIT,
             CHASSIS_ROLL_PID_IOUT_LIMIT,
             CHASSIS_ROLL_PID_P,
             CHASSIS_ROLL_PID_I,
             CHASSIS_ROLL_PID_D);

    // 自复位phi0角度PID，用于在自复位时将theta角度收拢到0附近
    pid_init(&chassis.chassis_selfhelp_phi0_pid_L,
                CHASSIS_SELFHELP_PHI0_PID_OUT_LIMIT,
             CHASSIS_SELFHELP_PHI0_PID_IOUT_LIMIT,
                     CHASSIS_SELFHELP_PHI0_PID_P,
                     CHASSIS_SELFHELP_PHI0_PID_I,
                    CHASSIS_SELFHELP_PHI0_PID_D);

    pid_init(&chassis.chassis_selfhelp_phi0_pid_R,
             CHASSIS_SELFHELP_PHI0_PID_OUT_LIMIT,
             CHASSIS_SELFHELP_PHI0_PID_IOUT_LIMIT,
             CHASSIS_SELFHELP_PHI0_PID_P,
             CHASSIS_SELFHELP_PHI0_PID_I,
             CHASSIS_SELFHELP_PHI0_PID_D);
}

/** 底盘初始化 **/
void chassis_init(void) {
    /** 底盘初始化为失能状态 **/
    chassis.chassis_ctrl_mode = CHASSIS_DISABLE;
    chassis.chassis_state = CHASSIS_FALL;
    chassis.selfhelp_done = false;

    /** 关节初始化 **/
    joint_init();

    /** 关节使能 **/
    joint_enable();

    /** 轮子初始化 **/
    wheel_init();

    /** 底盘PID参数初始化 **/
    chassis_pid_init();

    /** 惯导系统初始化 **/
    INS_Init();

    /** 速度估计滤波器初始化 **/
    // 卡尔曼滤波器
    Speed_EstimateKF_Init(&Speed_EstimateKF); // 速度-加速度估计卡尔曼滤波器

}

/** 设置底盘控制信息 **/
static void set_chassis_ctrl_info(void)
{
    /** 线速度设定 **/
    chassis.chassis_ctrl_info.v_m_per_s = (float) (remote_ctrl.rc.ch[CHASSIS_VX_CHANNEL]) * RC_TO_VX;;

    /** 转向设定 **/
    chassis.chassis_ctrl_info.yaw_rad -= (float) (remote_ctrl.rc.ch[CHASSIS_YAW_CHANNEL]) * (-RC_TO_YAW_INCREMENT);

    /** 目标腿长设定 **/
    chassis.chassis_ctrl_info.target_length = MIN_L0;

}

/** 读取遥控器开关，设置底盘模式 **/
static void set_chassis_mode(void) {

    if (switch_is_down(remote_ctrl.rc.s[RC_s_R])) { // 下位：失能
        chassis.chassis_last_ctrl_mode = chassis.chassis_ctrl_mode;
        chassis.chassis_ctrl_mode = CHASSIS_DISABLE;
    }
    else if (switch_is_mid(remote_ctrl.rc.s[RC_s_R])) { // 中位：使能
        chassis.chassis_last_ctrl_mode = chassis.chassis_ctrl_mode;
        chassis.chassis_ctrl_mode = CHASSIS_ENABLE;
    }
}

void chassis_remote_cmd(void) {
    set_chassis_mode();

    set_chassis_ctrl_info();
}

/** 从IMU获取姿态信息并做坐标变换 **/
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

//    USART_Vofa_Justfloat_Transmit(chassis.imu_reference.pitch_rad, chassis.imu_reference.roll_rad);
//    USART_Vofa_Justfloat_Transmit(chassis.imu_reference.yaw_total_rad, 0);

    /** 陀螺仪角速度赋值 **/
    // rad/s
    chassis.imu_reference.pitch_gyro = -DM_IMU.gyro[X];
    chassis.imu_reference.yaw_gyro = -DM_IMU.gyro[Z];
    chassis.imu_reference.roll_gyro = -DM_IMU.gyro[Y];

//    USART_Vofa_Justfloat_Transmit(chassis.imu_reference.pitch_gyro, 0);

    // m/s
    chassis.imu_reference.ax = -DM_IMU.accel[Y];
    chassis.imu_reference.ay = DM_IMU.accel[X];
    chassis.imu_reference.az = DM_IMU.accel[Z];

    /** 机体坐标系加速度转换到世界坐标系 **/
    Body_Accel_To_Earth();

}

/** 角度格式化 **/
// 将 theta 角度限制在 [-0.5PI, 1.5PI] 范围内
static float Theta_format(float Angle)
{
    while (Angle > 1.5f * PI)
        Angle -= 2.0f * PI;
    while (Angle < -0.5f * PI)
        Angle += 2.0f * PI;
    return Angle;
}

static void chassis_variable_update(float dt) {

    /********************* phi phi_dot ***********************/
    chassis.leg_L.state_variable_feedback.phi = chassis.imu_reference.pitch_rad;
    chassis.leg_R.state_variable_feedback.phi = chassis.imu_reference.pitch_rad;

    chassis.leg_L.state_variable_feedback.phi_dot = chassis.imu_reference.pitch_gyro;
    chassis.leg_R.state_variable_feedback.phi_dot = chassis.imu_reference.pitch_gyro;

    /********************* theta theta_dot ***********************/

    //1.theta
    float L_theta_raw = (chassis.leg_L.vmc.forward_kinematics.fk_phi.phi0 - chassis.imu_reference.pitch_rad) - PI / 2;
    float R_theta_raw = (chassis.leg_R.vmc.forward_kinematics.fk_phi.phi0 - chassis.imu_reference.pitch_rad) - PI / 2;

    // 角度限制在 [-0.5 PI, 1.5 PI]
    chassis.leg_L.state_variable_feedback.theta = Theta_format(L_theta_raw);
    chassis.leg_R.state_variable_feedback.theta = Theta_format(R_theta_raw);

    //2. theta_dot
    chassis.leg_L.state_variable_feedback.theta_dot_last = chassis.leg_L.state_variable_feedback.theta_dot;
    chassis.leg_R.state_variable_feedback.theta_dot_last = chassis.leg_R.state_variable_feedback.theta_dot;

    chassis.leg_L.state_variable_feedback.theta_dot = chassis.leg_L.vmc.forward_kinematics.fk_phi.d_phi0 - chassis.imu_reference.pitch_gyro;
    chassis.leg_R.state_variable_feedback.theta_dot = chassis.leg_R.vmc.forward_kinematics.fk_phi.d_phi0 - chassis.imu_reference.pitch_gyro;

//    USART_Vofa_Justfloat_Transmit(chassis.leg_L.state_variable_feedback.theta_dot, chassis.leg_R.state_variable_feedback.theta_dot);

    // 2.1 theta_ddot 通过微分计算
    // dt to be changed
    chassis.leg_L.state_variable_feedback.theta_ddot =
            (chassis.leg_L.state_variable_feedback.theta_dot - chassis.leg_L.state_variable_feedback.theta_dot_last) /
                    dt;
    chassis.leg_R.state_variable_feedback.theta_ddot =
            (chassis.leg_R.state_variable_feedback.theta_dot - chassis.leg_R.state_variable_feedback.theta_dot_last) /
                    dt;

    /********************* x x_dot ***********************/
    speed_calc(chassis_dt);

}

/** 底盘观测器更新 **/
static void chassis_observer_update(void) {
    /** 获取姿态传感器数据 **/
    get_IMU_info();

    /** 虚拟模型控制解算 **/
    vmc_calc(chassis_dt);

    /** 状态变量更新 **/
    chassis_variable_update(chassis_dt);
}

/** 摔倒检测 **/
static bool chassis_is_fall_enter(void)
{
    return    chassis.leg_L.vmc.forward_kinematics.fk_phi.phi0 > CHASSIS_FALL_PHI0_ENTER
           || chassis.leg_R.vmc.forward_kinematics.fk_phi.phi0 > CHASSIS_FALL_PHI0_ENTER;
}

static float chassis_angle_diff_rad(float target, float current)
{
    return atan2f(sinf(target - current), cosf(target - current));
}

static bool chassis_selfhelp_leg_finished(float target_l0)
{
    return    fabsf(chassis.leg_L.vmc.forward_kinematics.fk_L0.L0 - target_l0) < CHASSIS_SELFHELP_L0_HOLD_EXIT
           && fabsf(chassis.leg_R.vmc.forward_kinematics.fk_L0.L0 - target_l0) < CHASSIS_SELFHELP_L0_HOLD_EXIT;
}

static bool chassis_selfhelp_phi0_finished(float target_phi0)
{
    return    fabsf(chassis_angle_diff_rad(target_phi0, chassis.leg_L.vmc.forward_kinematics.fk_phi.phi0)) < CHASSIS_SELFHELP_PHI0_EXIT
           && fabsf(chassis_angle_diff_rad(target_phi0, chassis.leg_R.vmc.forward_kinematics.fk_phi.phi0)) < CHASSIS_SELFHELP_PHI0_EXIT;
}

bool leg_finished   = false;
bool theta_finished = false;
static bool chassis_selfhelp_target_reached(void)
{
    leg_finished = chassis_selfhelp_leg_finished(MIN_L0);
    theta_finished = chassis_selfhelp_phi0_finished(CHASSIS_SELFHELP_PHI0_TARGET);

    return leg_finished && theta_finished;
}

static bool chassis_is_balanced(void)
{
    return fabsf(chassis.imu_reference.pitch_rad) < CHASSIS_BALANCE_PITCH_EXIT;
}

static void chassis_state_update(void)
{
    switch (chassis.chassis_state)
    {
        case CHASSIS_NORMAL:
        {
            if (chassis_is_fall_enter())
            {
                chassis.chassis_state = CHASSIS_FALL;
                chassis.selfhelp_done = false;
                chassis_selfhelp_reset_pid_memory();
                chassis_selfhelp_exit_count = 0;
            }
            break;
        }

        case CHASSIS_FALL:
        {
            if (chassis_selfhelp_target_reached())
            {
                if (++chassis_selfhelp_exit_count >= CHASSIS_SELFHELP_EXIT_CONFIRM_COUNT)
                {
                    chassis.chassis_ctrl_info.v_m_per_s = 0.0f;
                    chassis.chassis_ctrl_info.target_length = MIN_L0;
                    chassis.chassis_ctrl_info.yaw_rad = chassis.imu_reference.yaw_total_rad;
                    chassis.chassis_state = CHASSIS_STAND_UP;
                    chassis_standup_tick = 0;
                }
            }
            else
            {
                chassis_selfhelp_exit_count = 0;
            }
            break;
        }

        case CHASSIS_STAND_UP:
        {
            if (chassis_is_balanced())
            {
                chassis.selfhelp_done = true;
                chassis.chassis_state = CHASSIS_NORMAL;
            }
            break;
        }
    }
}

static void chassis_selfhelp(void)
{
    chassis.leg_L.wheel_torque = 0.0f;
    chassis.leg_R.wheel_torque = 0.0f;

    chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point = 0.0f;
    chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point = 0.0f;
    chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point = 0.0f;
    chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point = 0.0f;

    chassis.chassis_ctrl_info.v_m_per_s = 0.0f;
    chassis.chassis_ctrl_info.target_length = MIN_L0;
    chassis.chassis_ctrl_info.yaw_rad = chassis.imu_reference.yaw_total_rad;

    float L_L0_dot_set = pid_calc(&chassis.leg_L.leg_pos_pid,
                                  chassis.leg_L.vmc.forward_kinematics.fk_L0.L0,
                                  MIN_L0,
                                  chassis_dt);

    float R_L0_dot_set = pid_calc(&chassis.leg_R.leg_pos_pid,
                                  chassis.leg_R.vmc.forward_kinematics.fk_L0.L0,
                                  MIN_L0,
                                  chassis_dt);

    chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point =
            pid_calc(&chassis.leg_L.leg_speed_pid,
                     chassis.leg_L.vmc.forward_kinematics.fk_L0.L0_dot,
                     L_L0_dot_set,
                     chassis_dt);

    chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point =
            pid_calc(&chassis.leg_R.leg_speed_pid,
                     chassis.leg_R.vmc.forward_kinematics.fk_L0.L0_dot,
                     R_L0_dot_set,
                     chassis_dt);

    chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point =
            pid_calc(&chassis.chassis_selfhelp_phi0_pid_L,
                     chassis.leg_L.vmc.forward_kinematics.fk_phi.phi0,
                     CHASSIS_SELFHELP_PHI0_TARGET,
                     chassis_dt);

    chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point =
            pid_calc(&chassis.chassis_selfhelp_phi0_pid_R,
                     chassis.leg_R.vmc.forward_kinematics.fk_phi.phi0,
                     CHASSIS_SELFHELP_PHI0_TARGET,
                     chassis_dt);

    chassis.steer_compensatory_torque = pid_calc(&chassis.chassis_leg_coordination_pid,
                                                 chassis.phi0_error,
                                                 0.0f,
                                                 chassis_dt);

    chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis.steer_compensatory_torque;
    chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point -= chassis.steer_compensatory_torque;

    vmc_forward_dynamics(&chassis.leg_L.vmc, &chassis_physical_config);
    vmc_forward_dynamics(&chassis.leg_R.vmc, &chassis_physical_config);

    chassis.leg_L.joint_F_torque = chassis.leg_L.vmc.forward_kinematics.T1_T4_set_point.E.T1_set_point;
    chassis.leg_L.joint_B_torque = chassis.leg_L.vmc.forward_kinematics.T1_T4_set_point.E.T4_set_point;
    chassis.leg_R.joint_F_torque = chassis.leg_R.vmc.forward_kinematics.T1_T4_set_point.E.T1_set_point;
    chassis.leg_R.joint_B_torque = chassis.leg_R.vmc.forward_kinematics.T1_T4_set_point.E.T4_set_point;

    VAL_LIMIT(chassis.leg_L.joint_F_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
    VAL_LIMIT(chassis.leg_L.joint_B_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
    VAL_LIMIT(chassis.leg_R.joint_F_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
    VAL_LIMIT(chassis.leg_R.joint_B_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
}

static void chassis_standup_joint_hold(void)
{
    float phi0_target = CHASSIS_SELFHELP_PHI0_TARGET;

    chassis.leg_L.wheel_torque = 0.0f;
    chassis.leg_R.wheel_torque = 0.0f;

    chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point = 0.0f;
    chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point = 0.0f;
    chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point = 0.0f;
    chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point = 0.0f;

    chassis.chassis_ctrl_info.v_m_per_s = 0.0f;
    chassis.chassis_ctrl_info.target_length = MIN_L0;
    chassis.chassis_ctrl_info.yaw_rad = chassis.imu_reference.yaw_total_rad;

    float L_L0_dot_set = pid_calc(&chassis.leg_L.leg_pos_pid,
                                  chassis.leg_L.vmc.forward_kinematics.fk_L0.L0,
                                  MIN_L0,
                                  chassis_dt);

    float R_L0_dot_set = pid_calc(&chassis.leg_R.leg_pos_pid,
                                  chassis.leg_R.vmc.forward_kinematics.fk_L0.L0,
                                  MIN_L0,
                                  chassis_dt);

    chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point =
            pid_calc(&chassis.leg_L.leg_speed_pid,
                     chassis.leg_L.vmc.forward_kinematics.fk_L0.L0_dot,
                     L_L0_dot_set,
                     chassis_dt);

    chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point =
            pid_calc(&chassis.leg_R.leg_speed_pid,
                     chassis.leg_R.vmc.forward_kinematics.fk_L0.L0_dot,
                     R_L0_dot_set,
                     chassis_dt);

    chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point =
            pid_calc(&chassis.chassis_selfhelp_phi0_pid_L,
                     chassis.leg_L.vmc.forward_kinematics.fk_phi.phi0,
                     phi0_target,
                     chassis_dt);

    chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point =
            pid_calc(&chassis.chassis_selfhelp_phi0_pid_R,
                     chassis.leg_R.vmc.forward_kinematics.fk_phi.phi0,
                     phi0_target,
                     chassis_dt);

    chassis.steer_compensatory_torque = pid_calc(&chassis.chassis_leg_coordination_pid,
                                                 chassis.phi0_error,
                                                 0.0f,
                                                 chassis_dt);

    chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis.steer_compensatory_torque;
    chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point -= chassis.steer_compensatory_torque;

    vmc_forward_dynamics(&chassis.leg_L.vmc, &chassis_physical_config);
    vmc_forward_dynamics(&chassis.leg_R.vmc, &chassis_physical_config);

    chassis.leg_L.joint_F_torque = chassis.leg_L.vmc.forward_kinematics.T1_T4_set_point.E.T1_set_point;
    chassis.leg_L.joint_B_torque = chassis.leg_L.vmc.forward_kinematics.T1_T4_set_point.E.T4_set_point;
    chassis.leg_R.joint_F_torque = chassis.leg_R.vmc.forward_kinematics.T1_T4_set_point.E.T1_set_point;
    chassis.leg_R.joint_B_torque = chassis.leg_R.vmc.forward_kinematics.T1_T4_set_point.E.T4_set_point;

    VAL_LIMIT(chassis.leg_L.joint_F_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
    VAL_LIMIT(chassis.leg_L.joint_B_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
    VAL_LIMIT(chassis.leg_R.joint_F_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
    VAL_LIMIT(chassis.leg_R.joint_B_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
}

static void wheel_calc(void) {
    /******************************* Wheel *************************************/

    /** 根据当前腿长拟合轮子LQR K矩阵 **/
    chassis_K_matrix_fitting(chassis.leg_L.vmc.forward_kinematics.fk_L0.L0, wheel_K_L, wheel_fitting_factor);
    chassis_K_matrix_fitting(chassis.leg_R.vmc.forward_kinematics.fk_L0.L0, wheel_K_R, wheel_fitting_factor);

    float target_yaw_speed = pid_calc(&chassis.chassis_turn_pos_pid,
                                      chassis.imu_reference.yaw_total_rad,
                                      chassis.chassis_ctrl_info.yaw_rad,
                                      chassis_dt);

    chassis.wheel_turn_torque = -pid_calc(&chassis.chassis_turn_speed_pid,
                                          chassis.imu_reference.yaw_gyro,
                                          target_yaw_speed,
                                          chassis_dt);


    float x_feedback = 0.5f * (chassis.leg_L.state_variable_feedback.x + chassis.leg_R.state_variable_feedback.x);
    float x_dot_feedback = 0.5f * (chassis.leg_L.state_variable_feedback.x_dot + chassis.leg_R.state_variable_feedback.x_dot);

    float x_ref = 0.5f * (chassis.leg_L.state_variable_ref.x + chassis.leg_R.state_variable_ref.x);

    float x_dot_ref = pid_calc(&chassis.chassis_wheel_x_pos_pid,
                         x_feedback,
                         x_ref,
                         chassis_dt);

//    if (fabsf(chassis.chassis_ctrl_info.v_m_per_s) > CHASSIS_WHEEL_X_PID_V_DEADBAND)
//    {
//        x_ref = x_feedback;
//
//        x_dot_ref = chassis.chassis_ctrl_info.v_m_per_s;
//    }
//    else
//    {
//        x_dot_ref = pid_calc(&chassis.chassis_wheel_x_pos_pid,
//                             x_feedback,
//                             x_ref,
//                             chassis_dt);
//    }

//    float wheel_x_torque = pid_calc(&chassis.chassis_wheel_x_speed_pid,
//                                    x_dot_feedback,
//                                    x_dot_ref,
//                                    chassis_dt);

    float wheel_x_torque = pid_calc(&chassis.chassis_wheel_x_speed_pid,
                                    x_dot_feedback,
                                    x_dot_ref,
                                    chassis_dt);

    // LQR 轮端，每个 K*state 结果存到 state_variable_wheel_out，方便 VOFA 查看
    chassis.leg_L.state_variable_wheel_out.theta     = wheel_K_L[0] * (chassis.leg_L.state_variable_feedback.theta - THETA_OFFSET);
    chassis.leg_L.state_variable_wheel_out.theta_dot = wheel_K_L[1] * (chassis.leg_L.state_variable_feedback.theta_dot - 0.0f);
    chassis.leg_L.state_variable_wheel_out.x         = 0.0f;
    chassis.leg_L.state_variable_wheel_out.x_dot     = 0.0f;
    chassis.leg_L.state_variable_wheel_out.phi       = wheel_K_L[4] * (chassis.leg_L.state_variable_feedback.phi - PHI_OFFSET);
    chassis.leg_L.state_variable_wheel_out.phi_dot   = wheel_K_L[5] * (chassis.leg_L.state_variable_feedback.phi_dot - 0.0f);

    chassis.leg_R.state_variable_wheel_out.theta     = wheel_K_R[0] * (chassis.leg_R.state_variable_feedback.theta - THETA_OFFSET);
    chassis.leg_R.state_variable_wheel_out.theta_dot = wheel_K_R[1] * (chassis.leg_R.state_variable_feedback.theta_dot - 0.0f);
    chassis.leg_R.state_variable_wheel_out.x         = 0.0f;
    chassis.leg_R.state_variable_wheel_out.x_dot     = 0.0f;
    chassis.leg_R.state_variable_wheel_out.phi       = wheel_K_R[4] * (chassis.leg_R.state_variable_feedback.phi - PHI_OFFSET);
    chassis.leg_R.state_variable_wheel_out.phi_dot   = wheel_K_R[5] * (chassis.leg_R.state_variable_feedback.phi_dot - 0.0f);

//    chassis.leg_L.wheel_torque = - chassis.leg_L.state_variable_wheel_out.theta
//                                 - chassis.leg_L.state_variable_wheel_out.theta_dot
//                                 - WHEEL_PHI_OUT * chassis.leg_L.state_variable_wheel_out.phi
//                                 - WHEEL_PHI_OUT * chassis.leg_L.state_variable_wheel_out.phi_dot
//                                 + wheel_x_torque;
//
//    chassis.leg_R.wheel_torque = - chassis.leg_R.state_variable_wheel_out.theta
//                                 - chassis.leg_R.state_variable_wheel_out.theta_dot
//                                 - WHEEL_PHI_OUT * chassis.leg_R.state_variable_wheel_out.phi
//                                 - WHEEL_PHI_OUT * chassis.leg_R.state_variable_wheel_out.phi_dot
//                                 + wheel_x_torque;

    chassis.leg_L.wheel_torque = wheel_x_torque;
    chassis.leg_R.wheel_torque = wheel_x_torque;


//    chassis.leg_L.wheel_torque -= chassis.wheel_turn_torque;
//    chassis.leg_R.wheel_torque += chassis.wheel_turn_torque;

    chassis.leg_L.wheel_torque *= -1;

    VAL_LIMIT(chassis.leg_L.wheel_torque, MIN_WHEEL_TORQUE, MAX_WHEEL_TORQUE);
    VAL_LIMIT(chassis.leg_R.wheel_torque, MIN_WHEEL_TORQUE, MAX_WHEEL_TORQUE);
}

/** 关节控制器解算 **/
static void chassis_standup_wheel_balance(void)
{
    chassis_K_matrix_fitting(chassis.leg_L.vmc.forward_kinematics.fk_L0.L0, wheel_K_L, wheel_fitting_factor);
    chassis_K_matrix_fitting(chassis.leg_R.vmc.forward_kinematics.fk_L0.L0, wheel_K_R, wheel_fitting_factor);

    float target_yaw_speed = pid_calc(&chassis.chassis_turn_pos_pid,
                                      chassis.imu_reference.yaw_total_rad,
                                      chassis.chassis_ctrl_info.yaw_rad,
                                      chassis_dt);

    chassis.wheel_turn_torque = -pid_calc(&chassis.chassis_turn_speed_pid,
                                          chassis.imu_reference.yaw_gyro,
                                          target_yaw_speed,
                                          chassis_dt);

    chassis.leg_L.state_variable_wheel_out.theta     =                                                                  0.0f ;
    chassis.leg_L.state_variable_wheel_out.theta_dot =                                                                  0.0f ;
    chassis.leg_L.state_variable_wheel_out.x         =                                                                  0.0f ;
    chassis.leg_L.state_variable_wheel_out.x_dot     =                                                                  0.0f ;
    chassis.leg_L.state_variable_wheel_out.phi       = wheel_K_L[4] * (chassis.leg_L.state_variable_feedback.phi     -  0.0f);
    chassis.leg_L.state_variable_wheel_out.phi_dot   = wheel_K_L[5] * (chassis.leg_L.state_variable_feedback.phi_dot -  0.0f);

    chassis.leg_R.state_variable_wheel_out.theta     =                                                                  0.0f ;
    chassis.leg_R.state_variable_wheel_out.theta_dot =                                                                  0.0f ;
    chassis.leg_R.state_variable_wheel_out.x         =                                                                  0.0f ;
    chassis.leg_R.state_variable_wheel_out.x_dot     =                                                                  0.0f ;
    chassis.leg_R.state_variable_wheel_out.phi       = wheel_K_R[4] * (chassis.leg_R.state_variable_feedback.phi     -  0.0f);
    chassis.leg_R.state_variable_wheel_out.phi_dot   = wheel_K_R[5] * (chassis.leg_R.state_variable_feedback.phi_dot -  0.0f);

    chassis.leg_L.wheel_torque = - chassis.leg_L.state_variable_wheel_out.phi
                                 - chassis.leg_L.state_variable_wheel_out.phi_dot;

    chassis.leg_R.wheel_torque = - chassis.leg_R.state_variable_wheel_out.phi
                                 - chassis.leg_R.state_variable_wheel_out.phi_dot;

    chassis.leg_L.wheel_torque -= chassis.wheel_turn_torque;
    chassis.leg_R.wheel_torque += chassis.wheel_turn_torque;

    chassis.leg_L.wheel_torque *= -1;

    VAL_LIMIT(chassis.leg_L.wheel_torque, MIN_WHEEL_TORQUE, MAX_WHEEL_TORQUE);
    VAL_LIMIT(chassis.leg_R.wheel_torque, MIN_WHEEL_TORQUE, MAX_WHEEL_TORQUE);
}

static void joint_calc(void) {
/******************************* Joint *************************************/

    chassis_K_matrix_fitting(chassis.leg_L.vmc.forward_kinematics.fk_L0.L0, joint_K_L, joint_fitting_factor);
    chassis_K_matrix_fitting(chassis.leg_R.vmc.forward_kinematics.fk_L0.L0, joint_K_R, joint_fitting_factor);

    /** Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp Tp **/

    /****** 双腿协调pid ******/
    chassis.steer_compensatory_torque = CHASSIS_LEG_COORDINATION_PID_P * (0.0f - chassis.phi0_error)
                                        + CHASSIS_LEG_COORDINATION_PID_D * (0.0f - chassis.d_phi0_error); // 双腿协调PD补偿

    float x_feedback = 0.5f * (chassis.leg_L.state_variable_feedback.x + chassis.leg_R.state_variable_feedback.x);
    float x_dot_feedback = 0.5f * (chassis.leg_L.state_variable_feedback.x_dot + chassis.leg_R.state_variable_feedback.x_dot);

    float x_ref = 0.5f * (chassis.leg_L.state_variable_ref.x + chassis.leg_R.state_variable_ref.x);

    float x_dot_ref = pid_calc(&chassis.chassis_joint_x_pos_pid,
                         x_feedback,
                         x_ref,
                         chassis_dt);


//    if (fabsf(chassis.chassis_ctrl_info.v_m_per_s) > CHASSIS_WHEEL_X_PID_V_DEADBAND)
//    {
//        x_ref = x_feedback;
//
//        x_dot_ref = chassis.chassis_ctrl_info.v_m_per_s;
//    }
//    else
//    {
//        x_dot_ref = pid_calc(&chassis.chassis_joint_x_pos_pid,
//                             x_feedback,
//                             x_ref,
//                             chassis_dt);
//    }

    float joint_x_torque = pid_calc(&chassis.chassis_joint_x_speed_pid,
                                    x_dot_feedback,
                                    x_dot_ref,
                                    chassis_dt);

    chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point =  joint_x_torque;
    chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point =  joint_x_torque;

//    chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis.steer_compensatory_torque;
//    chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point -= chassis.steer_compensatory_torque;

//    chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point =  - joint_K_L[0] * (chassis.leg_L.state_variable_feedback.theta - THETA_OFFSET)
//                                                                         - joint_K_L[1] * (chassis.leg_L.state_variable_feedback.theta_dot - 0.0f)
//                                                                         - 0.0f
//                                                                         - 0.0f
//                                                                         - joint_K_L[4] * (chassis.leg_L.state_variable_feedback.phi - PHI_OFFSET)
//                                                                         - joint_K_L[5] * (chassis.leg_L.state_variable_feedback.phi_dot - 0.0f)
//                                                                         - joint_x_torque;
//
//
//    chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point =  - joint_K_R[0] * (chassis.leg_R.state_variable_feedback.theta - THETA_OFFSET)
//                                                                         - joint_K_R[1] * (chassis.leg_R.state_variable_feedback.theta_dot - 0.0f)
//                                                                         - 0.0f
//                                                                         - 0.0f
//                                                                         - joint_K_R[4] * (chassis.leg_R.state_variable_feedback.phi - PHI_OFFSET)
//                                                                         - joint_K_R[5] * (chassis.leg_R.state_variable_feedback.phi_dot - 0.0f)
//                                                                         - joint_x_torque;
//
//    chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis.steer_compensatory_torque;
//    chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point -= chassis.steer_compensatory_torque;

    /** 关节Tp力矩使用LQR计算，包含theta/theta_dot/x/x_dot/phi/phi_dot六个状态反馈 **/

    // 左侧 Tp 符合 “力矩为正时，逆时针旋转”，而右侧 Tp 相反；
    // 不要在 Tp 直接加负号，在最终发送到电机时对每个电机的力矩加。

    /** End End End End End End End End End End End End End End End End End End End End End End End End **/


    /** F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F **/

    /****** Leg pid ******/

    float L_L0_dot_set = pid_calc(&chassis.leg_L.leg_pos_pid,
                                  chassis.leg_L.vmc.forward_kinematics.fk_L0.L0,
                                  MID_L0,
                                  chassis_dt);

    float R_L0_dot_set = pid_calc(&chassis.leg_R.leg_pos_pid,
                                  chassis.leg_R.vmc.forward_kinematics.fk_L0.L0,
                                  MID_L0,
                                  chassis_dt);

    pid_calc(&chassis.leg_L.leg_speed_pid,
             chassis.leg_L.vmc.forward_kinematics.fk_L0.L0_dot,
             L_L0_dot_set,
             chassis_dt);

    pid_calc(&chassis.leg_R.leg_speed_pid,
             chassis.leg_R.vmc.forward_kinematics.fk_L0.L0_dot,
             R_L0_dot_set,
             chassis_dt);

    /****** Roll pid ******/

    chassis.roll_compensatory_torque = CHASSIS_ROLL_PID_P * (0.0f - chassis.imu_reference.roll_rad)
                                       + CHASSIS_ROLL_PID_D * (0.0f - chassis.imu_reference.roll_gyro); // 双腿协调PD补偿

    chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point = 0.0f;

    chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point = 0.0f;

//    chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point = chassis.leg_L.leg_speed_pid.out
//            + 0.5f * chassis_physical_config.body_weight * GRAVITY * cosf(chassis.leg_L.state_variable_feedback.theta)
//            + chassis.roll_compensatory_torque;
//
//    chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point = chassis.leg_R.leg_speed_pid.out
//           + 0.5f * chassis_physical_config.body_weight * GRAVITY * cosf(chassis.leg_R.state_variable_feedback.theta)
//           - chassis.roll_compensatory_torque;




    /** End End End End End End End End End End End End End End End End End End End End End End End End **/

    // 通过VMC解算关节力矩
    vmc_forward_dynamics(&chassis.leg_L.vmc, &chassis_physical_config);
    vmc_forward_dynamics(&chassis.leg_R.vmc, &chassis_physical_config);

    chassis.leg_L.joint_F_torque = chassis.leg_L.vmc.forward_kinematics.T1_T4_set_point.E.T1_set_point;//F
    chassis.leg_L.joint_B_torque = chassis.leg_L.vmc.forward_kinematics.T1_T4_set_point.E.T4_set_point;//B

    chassis.leg_R.joint_F_torque = chassis.leg_R.vmc.forward_kinematics.T1_T4_set_point.E.T1_set_point;//F
    chassis.leg_R.joint_B_torque = chassis.leg_R.vmc.forward_kinematics.T1_T4_set_point.E.T4_set_point;//B

    // 力矩限幅
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
    chassis.leg_L.state_variable_feedback.x_dot = 0.0f;
    chassis.leg_R.state_variable_feedback.x_dot = 0.0f;
    chassis.leg_L.state_variable_ref.x = 0.0f;
    chassis.leg_R.state_variable_ref.x = 0.0f;
    chassis.leg_L.state_variable_ref.x_dot = 0.0f;
    chassis.leg_R.state_variable_ref.x_dot = 0.0f;
    chassis.leg_L.state_variable_error.x = 0.0f;
    chassis.leg_R.state_variable_error.x = 0.0f;
    chassis.leg_L.state_variable_error.x_dot = 0.0f;
    chassis.leg_R.state_variable_error.x_dot = 0.0f;
    CHASSIS_CLEAR_PID_STATE(chassis.chassis_wheel_x_pos_pid);
    CHASSIS_CLEAR_PID_STATE(chassis.chassis_wheel_x_speed_pid);
    CHASSIS_CLEAR_PID_STATE(chassis.chassis_joint_x_pos_pid);
    CHASSIS_CLEAR_PID_STATE(chassis.chassis_joint_x_speed_pid);

    chassis.chassis_ctrl_info.yaw_rad = chassis.imu_reference.yaw_total_rad;

    chassis.chassis_ctrl_info.target_length = MIN_L0;

    /** 清除恢复标志 **/
//    chassis.chassis_recover_finish = false;
    chassis.chassis_state = CHASSIS_FALL;
    chassis.selfhelp_done = false;
    chassis_selfhelp_exit_count = 0;
}

static void chassis_standup_balance_task(void)
{
    chassis_standup_joint_hold();
    chassis_standup_wheel_balance();
}
/** 底盘使能任务 **/
static void chassis_enable_task(void)
{
    /** 状态机调度                               **/
//    chassis_state_update();
    chassis.chassis_state = CHASSIS_NORMAL;

    // 摔倒状态
    if (chassis.chassis_state == CHASSIS_FALL)
    {
        // 自复位过程中通过腿长和theta控制让底盘恢复站立
        chassis_selfhelp();
        return;
    }

    if (chassis.chassis_state == CHASSIS_STAND_UP)
    {
        // 轮子LQR + 关节theta LQR / 腿长PID / 最小腿长目标
        chassis_standup_balance_task();
        return;
    }

    wheel_calc();
    joint_calc();
}
/** 电机MIT模式-发送力矩指令 **/
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
    VAL_LIMIT(wheel_L_torque, MIN_WHEEL_TORQUE, MAX_WHEEL_TORQUE);
    VAL_LIMIT(wheel_R_torque, MIN_WHEEL_TORQUE, MAX_WHEEL_TORQUE);

    int16_t L_wheel_data = wheel_L_torque * (1 / TORQUE_CONSTANT_3508) * DATA_PER_A;
    int16_t R_wheel_data = wheel_R_torque * (1 / TORQUE_CONSTANT_3508) * DATA_PER_A;

    DJI_Current_Set(L_wheel_data,
                    R_wheel_data,
                    0,
                    0);

}

void chassis_task(void) {

    // 计算时间间隔，确保dt在合理范围内
    chassis_dt = DWT_GetDeltaT(&chassis_dwt_cnt);
    if (chassis_dt <= 0.000001f || chassis_dt > 0.1f)
    {
        chassis_dt = CHASSIS_PERIOD * 0.001f;
    }

    /** 遥控器指令更新(模式 + 控制量) **/
    chassis_remote_cmd();

    /** 底盘观测器更新 **/
    chassis.chassis_state = CHASSIS_NORMAL;

    chassis_observer_update();

    switch (chassis.chassis_ctrl_mode) {
        case CHASSIS_DISABLE: {
            chassis_disable_task();
            break;
        }

        case CHASSIS_ENABLE: {
            chassis_enable_task();
            break;
        }

        default: {
            chassis_disable_task();
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

//    MIT_send_torque_task(0,
//                         0,
//                         0,
//                         0,
//                         chassis.leg_L.wheel_torque,
//                         chassis.leg_R.wheel_torque,
//                         0,
//                         0,
//                         0,
//                         0);

//    MIT_send_torque_task(chassis.leg_L.joint_F_torque,
//                         chassis.leg_L.joint_B_torque,
//                         0,
//                         0,
//                         0,
//                         0,
//                         0,
//                         0,
//                         0,
//                         0);

//    MIT_send_torque_task(chassis.leg_L.joint_F_torque,
//                         chassis.leg_L.joint_B_torque,
//                         -chassis.leg_R.joint_F_torque,
//                         -chassis.leg_R.joint_B_torque,
//                         0,
//                         0,
//                         0,
//                         0,
//                         0,
//                         0);

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

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

// Stage0：当腿当前姿态不适合直接收腿时，先把腿调整到一个容易复位的位置
// Stage1：收腿、腿摆角复位
typedef enum {
    CHASSIS_SELFHELP_RESET_STAGE0 = 0,
    CHASSIS_SELFHELP_RESET_STAGE1,
} ChassisSelfhelpResetStage;

static ChassisSelfhelpResetStage chassis_selfhelp_stage = CHASSIS_SELFHELP_RESET_STAGE1;

// 自救阶段初始化
static bool chassis_selfhelp_sequence_ready = false;

// 左右腿是否需要先进行 Stage0 自救
static bool chassis_selfhelp_l_need_stage0 = false;
static bool chassis_selfhelp_r_need_stage0 = false;

// 自救 Stage1 完成到 Stand_up 的确认时间
static uint16_t chassis_selfhelp_exit_count = 0;

//
static uint16_t chassis_standup_tick = 0;
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

    // 倒地自救摆角PID：倒地后控制左右腿theta回到0度附近
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
    /** 初始化底盘模式为失能 **/
    chassis.chassis_ctrl_mode = CHASSIS_DISABLE;
    chassis.chassis_state = CHASSIS_FALL;

    /** 关节电机初始化 **/
    joint_init();

    /** 关节电机使能 **/
    joint_enable();

    /** 轮毂电机初始化 **/
    wheel_init();

    /** 底盘pid初始化 **/
    chassis_pid_init();

    /** 姿态解算初始化 **/
    INS_Init();

    /** 滤波器初始化 **/
    // 卡尔曼滤波
    Speed_EstimateKF_Init(&Speed_EstimateKF); // 速度-加速度融合观测器

}

/** 底盘接收遥控器信息 **/
static void set_chassis_ctrl_info(void)
{
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
    }
    else if (switch_is_mid(remote_ctrl.rc.s[RC_s_R])) { // 使能
        chassis.chassis_last_ctrl_mode = chassis.chassis_ctrl_mode;
        chassis.chassis_ctrl_mode = CHASSIS_ENABLE;
    }
}

void chassis_remote_cmd(void) {
    set_chassis_mode();

    set_chassis_ctrl_info();
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

//    USART_Vofa_Justfloat_Transmit(chassis.imu_reference.pitch_rad, chassis.imu_reference.roll_rad);
//    USART_Vofa_Justfloat_Transmit(chassis.imu_reference.yaw_total_rad, 0);

    /** 更新各轴加速度和角速度 **/
    // rad/s
    chassis.imu_reference.pitch_gyro = -DM_IMU.gyro[X];
    chassis.imu_reference.yaw_gyro = -DM_IMU.gyro[Z];
    chassis.imu_reference.roll_gyro = -DM_IMU.gyro[Y];

//    USART_Vofa_Justfloat_Transmit(chassis.imu_reference.pitch_gyro, 0);

    // m/s
    chassis.imu_reference.ax = -DM_IMU.accel[Y];
    chassis.imu_reference.ay = DM_IMU.accel[X];
    chassis.imu_reference.az = DM_IMU.accel[Z];

    /** 将机体坐标系的加速度转换为世界坐标系 **/
    Body_Accel_To_Earth();

}

/** 更新底盘变量 **/
// 把 theta 突变点置于：腿在前
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

    // 归一化到 [-0.5 PI, 1.5 PI]
    // chassis.leg_L.state_variable_feedback.theta = L_theta_raw;
    // chassis.leg_R.state_variable_feedback.theta = R_theta_raw;
    chassis.leg_L.state_variable_feedback.theta = Theta_format(L_theta_raw);
    chassis.leg_R.state_variable_feedback.theta = Theta_format(R_theta_raw);

//    USART_Vofa_Justfloat_Transmit(chassis.leg_L.state_variable_feedback.theta, chassis.leg_R.state_variable_feedback.theta);

    //2. theta_dot
    chassis.leg_L.state_variable_feedback.theta_dot_last = chassis.leg_L.state_variable_feedback.theta_dot;
    chassis.leg_R.state_variable_feedback.theta_dot_last = chassis.leg_R.state_variable_feedback.theta_dot;

    chassis.leg_L.state_variable_feedback.theta_dot = chassis.leg_L.vmc.forward_kinematics.fk_phi.d_phi0 + chassis.imu_reference.pitch_gyro;
    chassis.leg_R.state_variable_feedback.theta_dot = chassis.leg_R.vmc.forward_kinematics.fk_phi.d_phi0 + chassis.imu_reference.pitch_gyro;

//    USART_Vofa_Justfloat_Transmit(chassis.leg_L.state_variable_feedback.theta_dot, chassis.leg_R.state_variable_feedback.theta_dot);

    // 2.1 theta_ddot 需要加低通滤波
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
    /** 获取传感器数据 **/
    get_IMU_info();

    /** 更新五连杆参数 **/
    vmc_calc(chassis_dt);

    /** 更新底盘变量 **/
    chassis_variable_update(chassis_dt);
}

/** 倒地状态判断 **/
static bool chassis_is_fall_enter(void)
{
    return    fabsf(chassis.imu_reference.pitch_rad            ) > CHASSIS_FALL_ATTITUDE_ENTER // pitch
           || fabsf(chassis.imu_reference.roll_rad             ) > CHASSIS_FALL_ATTITUDE_ENTER // roll
           || fabsf(chassis.leg_L.state_variable_feedback.theta) >    CHASSIS_FALL_THETA_ENTER // theta
           || fabsf(chassis.leg_R.state_variable_feedback.theta) >    CHASSIS_FALL_THETA_ENTER;
}

static float chassis_angle_diff_rad(float target, float current)
{
    return atan2f(sinf(target - current), cosf(target - current));
}

static bool chassis_selfhelp_need_stage0(Leg *leg)
{
    float theta = leg->state_variable_feedback.theta;
    float phi1 = leg->vmc.forward_kinematics.fk_phi.phi1;
    float L0 = leg->vmc.forward_kinematics.fk_L0.L0;

    return    theta > 0.5f * PI
           || phi1 < 0.5f * PI
           || (L0 > MIN_L0 + CHASSIS_SELFHELP_STAGE0_L0_MARGIN && phi1 > PI);
}

// 自救阶段初始化：判断是否需要先进入 Stage0 阶段自救
static void chassis_selfhelp_begin_sequence(void)
{
    /** chassis_selfhelp_need_stage0()  **/
    /** 判断左右腿是否需要 Stage0 自救阶段  **/
    chassis_selfhelp_l_need_stage0 = chassis_selfhelp_need_stage0(&chassis.leg_L);
    chassis_selfhelp_r_need_stage0 = chassis_selfhelp_need_stage0(&chassis.leg_R);

    // 如果左腿或右腿需要 Stage0，则先进入 Stage0
    chassis_selfhelp_stage = (chassis_selfhelp_l_need_stage0 || chassis_selfhelp_r_need_stage0)
                             ? CHASSIS_SELFHELP_RESET_STAGE0
                             : CHASSIS_SELFHELP_RESET_STAGE1;

    chassis_selfhelp_exit_count = 0;
    chassis_selfhelp_sequence_ready = true;
}

// 根据自救阶段给期望腿长赋值
// Stage0：  0.3 m
// Stage1：  0.16m
static float chassis_selfhelp_stage_l0_target(void) {
    return (chassis_selfhelp_stage == CHASSIS_SELFHELP_RESET_STAGE0) ? CHASSIS_SELFHELP_L0_STAGE0
                                                                     : CHASSIS_SELFHELP_L0_STAGE1;
}

// 根据自救阶段给期望 theta 赋值
// Stage0：  0.36 PI
// Stage1：  0
static float chassis_selfhelp_stage_theta_target(void) {
    return (chassis_selfhelp_stage == CHASSIS_SELFHELP_RESET_STAGE0) ? CHASSIS_SELFHELP_THETA_STAGE0
                                                                     : CHASSIS_SELFHELP_THETA_STAGE1;
}

/** 腿长复位完成 **/
static bool chassis_selfhelp_leg_finished(float target_l0) {
    return    fabsf(chassis.leg_L.vmc.forward_kinematics.fk_L0.L0 - target_l0) < CHASSIS_SELFHELP_L0_EXIT
           && fabsf(chassis.leg_R.vmc.forward_kinematics.fk_L0.L0 - target_l0) < CHASSIS_SELFHELP_L0_EXIT;
}

/** theta复位完成 **/
static bool chassis_selfhelp_theta_finished(float target_theta)
{
    return    fabsf(chassis_angle_diff_rad(target_theta, chassis.leg_L.state_variable_feedback.theta)) < CHASSIS_SELFHELP_THETA_EXIT
           && fabsf(chassis_angle_diff_rad(target_theta, chassis.leg_R.state_variable_feedback.theta)) < CHASSIS_SELFHELP_THETA_EXIT;
}

/** 通过 期望腿长 和 期望 theta 判断某一阶段是否完成 **/
bool leg_finished   = false;
bool theta_finished = false;
static bool chassis_selfhelp_target_reached(void)
{
    leg_finished = chassis_selfhelp_leg_finished(    chassis_selfhelp_stage_l0_target()        );  // 期望 腿长
    theta_finished = chassis_selfhelp_theta_finished(chassis_selfhelp_stage_theta_target()     ); // 期望 theta

    return   leg_finished  // 期望 腿长
          && theta_finished; // 期望 theta
}
static void chassis_state_update(void)
{
    switch (chassis.chassis_state)
    {
        case CHASSIS_NORMAL:
        {
            // 检测是否倒地
            if (chassis_is_fall_enter())
            {
                // 正常运行中检测到倒地，切入自救状态并初始化复位阶段
                chassis.chassis_state = CHASSIS_FALL;
                chassis_selfhelp_begin_sequence();
            }
            break;
        }

        // 倒地状态
        case CHASSIS_FALL:
        {

            if (!chassis_selfhelp_sequence_ready)
            {
                /** 首次进入 FALL 状态，判断先进入哪个阶段：State0 / Stage1 **/
                chassis_selfhelp_begin_sequence();
            }

            /** 某一阶段完成后，状态切换                      **/
            /** 通过 期望腿长 和 期望 theta 判断某一阶段是否完成 **/
            if (chassis_selfhelp_target_reached())
            {
                /** Stage0 完成后，-> Stage1 **/
                if (chassis_selfhelp_stage == CHASSIS_SELFHELP_RESET_STAGE0)
                {
                    chassis_selfhelp_stage = CHASSIS_SELFHELP_RESET_STAGE1;
                    chassis_selfhelp_exit_count = 0;
                }

                /** Stage1 完成后 100ms，-> Stand_up **/
                else if (++chassis_selfhelp_exit_count >= CHASSIS_SELFHELP_EXIT_CONFIRM_COUNT)
                {
                    // stage1复位完成后，进入STAND_UP，继续保持Stage1目标用于验证切换
                    chassis.chassis_state = CHASSIS_STAND_UP;

                    chassis_standup_tick = 0;

                    // 重置自救标志位
                    chassis_selfhelp_sequence_ready = false;

                    chassis.chassis_ctrl_info.v_m_per_s = 0.0f;
                    chassis.chassis_ctrl_info.target_length = MIN_L0;
                    chassis.chassis_ctrl_info.yaw_rad = chassis.imu_reference.yaw_total_rad;
                }
            }
            /** Stage0 或 Stage1 尚未完成，继续执行 **/
            else
            {
                chassis_selfhelp_exit_count = 0;
            }
            break;
        }

        case CHASSIS_STAND_UP:
        {
            // STAND_UP中轮毂LQR + 关节theta LQR/防劈叉/MIN腿长控制
            break;
        }
    }
}

// Stage1：清除轮毂、关节输入，根据自救阶段获取 期望腿长 和 期望theta
// Stage2：F  闭环控制腿长
// Stage3：Tp 闭环控制 theta，协调双腿
float target_l0 = 0.0f;
float target_theta = 0.0f;
static void chassis_selfhelp(void)
{
    /** 倒地时轮毂不参与平衡                         **/
    chassis.leg_L.wheel_torque = 0.0f;
    chassis.leg_R.wheel_torque = 0.0f;

    /** 重置关节输入                                **/
    chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point = 0.0f;
    chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point = 0.0f;
    chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point = 0.0f;
    chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point = 0.0f;

    /** 按自救阶段获取期望腿长                        **/
    target_l0 = chassis_selfhelp_stage_l0_target();
    chassis.chassis_ctrl_info.target_length = target_l0;

    /** 按自救阶段获取期望 theta                     **/
    target_theta = chassis_selfhelp_stage_theta_target();

    /** 锁住当前yaw目标                             **/
    chassis.chassis_ctrl_info.yaw_rad = chassis.imu_reference.yaw_total_rad;

    /** F                                          **/
    /** 腿长串级PID，对不同 Stage 获取的期望腿长进行控制 **/
    /** 位置环                                      **/
    float L_L0_dot_set = pid_calc(&chassis.leg_L.leg_pos_pid,
                                  chassis.leg_L.vmc.forward_kinematics.fk_L0.L0,
                                  target_l0,
                                  chassis_dt);

    float R_L0_dot_set = pid_calc(&chassis.leg_R.leg_pos_pid,
                                  chassis.leg_R.vmc.forward_kinematics.fk_L0.L0,
                                  target_l0,
                                  chassis_dt);

    /** 速度环                                      **/
    chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point = pid_calc(&chassis.leg_L.leg_speed_pid,
                                                                                 chassis.leg_L.vmc.forward_kinematics.fk_L0.L0_dot,
                                                                                 L_L0_dot_set,
                                                                                 chassis_dt);

    chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Fy_set_point = pid_calc(&chassis.leg_R.leg_speed_pid,
                                                                                 chassis.leg_R.vmc.forward_kinematics.fk_L0.L0_dot,
                                                                                 R_L0_dot_set,
                                                                                 chassis_dt);

    /** Tp                                         **/
    /** 对 theta 闭环控制                            **/
    float L_theta_err = chassis_angle_diff_rad(target_theta,
                                               chassis.leg_L.state_variable_feedback.theta);
    float R_theta_err = chassis_angle_diff_rad(target_theta,
                                               chassis.leg_R.state_variable_feedback.theta);

    chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point = pid_calc(&chassis.chassis_selfhelp_phi0_pid_L,
                                                                                 chassis.leg_L.state_variable_feedback.theta,
                                                                                 chassis.leg_L.state_variable_feedback.theta + L_theta_err,
                                                                                 chassis_dt);

    chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point = pid_calc(&chassis.chassis_selfhelp_phi0_pid_R,
                                                                                 chassis.leg_R.state_variable_feedback.theta,
                                                                                 chassis.leg_R.state_variable_feedback.theta + R_theta_err,
                                                                                 chassis_dt);

    /** 协调左右腿                                      **/
    chassis.steer_compensatory_torque = CHASSIS_LEG_COORDINATION_PID_P * (0.0f - chassis.phi0_error)
                                        + CHASSIS_LEG_COORDINATION_PID_D * (0.0f - chassis.d_phi0_error); // 注意微分项正负

    chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis.steer_compensatory_torque;
    chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point -= chassis.steer_compensatory_torque;

    // 通过VMC把 ( Tp, F ) 转换为四个关节电机力矩
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

/** 计算驱动轮力矩 **/
static void wheel_calc(void) {
    /******************************* Wheel *************************************/

    /** 根据腿长和三次拟合系数拟合出反馈增益K **/
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

    // 轮毂LQR：根据当前腿长拟合出的K矩阵，对theta/x/phi等状态反馈输出轮毂力矩
    chassis.leg_L.wheel_torque = - wheel_K_L[0] * (chassis.leg_L.state_variable_feedback.theta - THETA_OFFSET)
                                 //- wheel_K_L[1] * (chassis.leg_L.state_variable_feedback.theta_dot - 0.0f)
                                 //+ wheel_K_L[2] * (chassis.leg_L.state_variable_error.x)
                                 //+ wheel_K_L[3] * (chassis.leg_L.state_variable_error.x_dot)
                                 //- wheel_K_L[4] * (chassis.leg_L.state_variable_feedback.phi - 0.0f)
                                 //- wheel_K_L[5] * (chassis.leg_L.state_variable_feedback.phi_dot - 0.0f)
                                 ;

    chassis.leg_R.wheel_torque = - wheel_K_R[0] * (chassis.leg_R.state_variable_feedback.theta - THETA_OFFSET)
                                 //- wheel_K_R[1] * (chassis.leg_R.state_variable_feedback.theta_dot - 0.0f)
                                 //+ wheel_K_R[2] * (chassis.leg_R.state_variable_error.x)
                                 //+ wheel_K_R[3] * (chassis.leg_R.state_variable_error.x_dot)
                                 //- wheel_K_R[4] * (chassis.leg_R.state_variable_feedback.phi - 0.0f)
                                 //- wheel_K_R[5] * (chassis.leg_R.state_variable_feedback.phi_dot - 0.0f)
                                 ;

    chassis.leg_L.wheel_torque -= chassis.wheel_turn_torque;
    chassis.leg_R.wheel_torque += chassis.wheel_turn_torque;

    chassis.leg_L.wheel_torque *= -1;

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

    chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point =
            -joint_K_L[0] * (chassis.leg_L.state_variable_feedback.theta - THETA_OFFSET)
            - joint_K_L[1] * (chassis.leg_L.state_variable_feedback.theta_dot - 0.0f);

    chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point =
            -joint_K_R[0] * (chassis.leg_R.state_variable_feedback.theta - THETA_OFFSET)
            - joint_K_R[1] * (chassis.leg_R.state_variable_feedback.theta_dot - 0.0f);

//    chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point =  - joint_K_L[0] * (chassis.leg_L.state_variable_feedback.theta - THETA_OFFSET)
//                                                                         - joint_K_L[1] * (chassis.leg_L.state_variable_feedback.theta_dot - 0.0f)
//                                                                         - joint_K_L[2] * (chassis.leg_L.state_variable_error.x)
//                                                                         - joint_K_L[3] * (chassis.leg_L.state_variable_error.x_dot)
//                                                                         - joint_K_L[4] * (chassis.leg_L.state_variable_feedback.phi - 0.0f)
//                                                                         - joint_K_L[5] * (chassis.leg_L.state_variable_feedback.phi_dot - 0.0f);
//
//
//    chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point =  - joint_K_R[0] * (chassis.leg_R.state_variable_feedback.theta - THETA_OFFSET)
//                                                                         - joint_K_R[1] * (chassis.leg_R.state_variable_feedback.theta_dot - 0.0f)
//                                                                         - joint_K_R[2] * (chassis.leg_R.state_variable_error.x)
//                                                                         - joint_K_R[3] * (chassis.leg_R.state_variable_error.x_dot)
//                                                                         - joint_K_R[4] * (chassis.leg_R.state_variable_feedback.phi - 0.0f)
//                                                                         - joint_K_R[5] * (chassis.leg_R.state_variable_feedback.phi_dot - 0.0f);
//
//    chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point += chassis.steer_compensatory_torque;
//    chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point -= chassis.steer_compensatory_torque;

//    chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point = 0.0f;
//    chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point = 0.0f;

    /** 不要在Tp层面修改左右侧旋转方向相反这个问题 因为解算出来加到电机上的力矩是对的 **/

    /** End End End End End End End End End End End End End End End End End End End End End End End End **/


    /** F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F **/

    /****** Leg pid ******/

    float L_L0_dot_set = pid_calc(&chassis.leg_L.leg_pos_pid,
                                  chassis.leg_L.vmc.forward_kinematics.fk_L0.L0,
                                  chassis.chassis_ctrl_info.target_length,
                                  chassis_dt);

    float R_L0_dot_set = pid_calc(&chassis.leg_R.leg_pos_pid,
                                  chassis.leg_R.vmc.forward_kinematics.fk_L0.L0,
                                  chassis.chassis_ctrl_info.target_length,
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

    chassis.chassis_ctrl_info.yaw_rad = chassis.imu_reference.yaw_total_rad;

    chassis.chassis_ctrl_info.target_length = MIN_L0;

    /** 初始化标志位 **/
//    chassis.chassis_recover_finish = false;
    chassis.chassis_state = CHASSIS_FALL;
    chassis_selfhelp_sequence_ready = false;
    chassis_selfhelp_exit_count = 0;
    chassis_standup_tick = 0;
}

static void chassis_standup_joint_lqr_task(void)
{
    chassis_K_matrix_fitting(chassis.leg_L.vmc.forward_kinematics.fk_L0.L0, joint_K_L, joint_fitting_factor);
    chassis_K_matrix_fitting(chassis.leg_R.vmc.forward_kinematics.fk_L0.L0, joint_K_R, joint_fitting_factor);

    chassis.steer_compensatory_torque = CHASSIS_LEG_COORDINATION_PID_P * (0.0f - chassis.phi0_error)
                                        + CHASSIS_LEG_COORDINATION_PID_D * (0.0f - chassis.d_phi0_error);


    chassis.leg_L.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point =  //0
                                                                            - joint_K_L[0] * (chassis.leg_L.state_variable_feedback.theta - THETA_OFFSET)
                                                                         //- joint_K_L[1] * (chassis.leg_L.state_variable_feedback.theta_dot - 0.0f)
                                                                         //-joint_K_L[2] * (chassis.leg_L.state_variable_error.x)
                                                                         //- joint_K_L[3] * (chassis.leg_L.state_variable_error.x_dot)
                                                                         //- joint_K_L[4] * (chassis.leg_L.state_variable_feedback.phi - 0.0f)
                                                                         //- joint_K_L[5] * (chassis.leg_L.state_variable_feedback.phi_dot - 0.0f)
                                                                         //+ chassis.steer_compensatory_torque
    ;


    chassis.leg_R.vmc.forward_kinematics.Fxy_set_point.E.Tp_set_point = //0
                                                                            - joint_K_R[0] * (chassis.leg_R.state_variable_feedback.theta - THETA_OFFSET)
                                                                         //- joint_K_R[1] * (chassis.leg_R.state_variable_feedback.theta_dot - 0.0f)
                                                                         //- joint_K_R[2] * (chassis.leg_R.state_variable_error.x)
                                                                         //- joint_K_R[3] * (chassis.leg_R.state_variable_error.x_dot)
                                                                         //- joint_K_R[4] * (chassis.leg_R.state_variable_feedback.phi - 0.0f)
                                                                            //-joint_K_R[5] * (chassis.leg_R.state_variable_feedback.phi_dot - 0.0f)
                                                                         //- chassis.steer_compensatory_torque
                                                                         ;



    // float L_L0_dot_set=0;
    // float R_L0_dot_set =0;
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
static void chassis_standup_balance_task(void)
{
    // STAND_UP：轮毂LQR + 关节theta LQR/防劈叉/MIN腿长控制
    chassis.chassis_ctrl_info.v_m_per_s = 0.0f;
    chassis.chassis_ctrl_info.target_length = MIN_L0;
    chassis.chassis_ctrl_info.yaw_rad = chassis.imu_reference.yaw_total_rad;

    wheel_calc();
    chassis_standup_joint_lqr_task();
}
/** 底盘使能任务 **/
static void chassis_enable_task(void)
{
    /** 底盘状态更新                            **/
    /** 自救 Fall  （Stage0 Stage1）、Stand_up) **/
    /** 平衡 Normal                            **/
    chassis_state_update();

    // 状态机
    if (chassis.chassis_state == CHASSIS_FALL)
    {
        // 倒地状态只执行收腿和theta回零，跳过正常轮腿控制计算
        chassis_selfhelp();
        return;
    }

    if (chassis.chassis_state == CHASSIS_STAND_UP)
    {
        // 轮毂LQR + 关节theta LQR/防劈叉/MIN腿长控制
        chassis_standup_balance_task();
        return;
    }

//    /** 正常运行：LQR轮毂 + 关节控制 **/
    wheel_calc();
    joint_calc();
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

    // 获取两次底盘任务执行之间的真实时间间隔，单位：s
    chassis_dt = DWT_GetDeltaT(&chassis_dwt_cnt);
    if (chassis_dt <= 0.000001f || chassis_dt > 0.1f)
    {
        chassis_dt = CHASSIS_PERIOD * 0.001f;
    }

    /** 获取底盘遥控器信息(模式 + 数据) **/
    chassis_remote_cmd();

    /** 底盘观测器更新 **/
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

//    MIT_send_torque_task(0,
//                         0,
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

    // chassis.leg_L.joint_F_torque=0;
    // chassis.leg_L.joint_B_torque=0;
    // chassis.leg_R.joint_F_torque=0;
    // chassis.leg_R.joint_B_torque=0;
    chassis.leg_L.wheel_torque=0;
    chassis.leg_R.wheel_torque=0;
    MIT_send_torque_task(chassis.leg_L.joint_F_torque,
                         chassis.leg_L.joint_B_torque,
                         -chassis.leg_R.joint_F_torque,
                         -chassis.leg_R.joint_B_torque,
                         chassis.leg_L.wheel_torque,
                         chassis.leg_R.wheel_torque,
                         0,
                         0,
                         0,
                         0);

}
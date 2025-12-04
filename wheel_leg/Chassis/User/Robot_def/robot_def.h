#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include <stdbool.h>

#include "pid.h"
#include "moving_filter.h"
#include "can_device.h"
#include "user_lib.h"
#include "DJI_motor.h"
#include "board_communication_task.h"
#include "low_pass_filter.h"


/*******************************************************************************
 *                                    底盘                                     *
 *******************************************************************************/

/** 宏定义 **/

// 底盘运行周期
#define CHASSIS_PERIOD 5 // ms 计算频率: 200Hz

// 平衡点
#define PHI_BALANCE 0.0f * DEGREE_TO_RAD // 平衡点在0.0°

// 旋转速度
#define SPIN_SPEED 5.0f

// 机体倾角
#define NOT_BALANCE_RAD 13.0f * DEGREE_TO_RAD
#define RECOVER_RAD 5.0f * DEGREE_TO_RAD

// 腿摆角
#define LEG_NORMAL_RAD 30.0f * DEGREE_TO_RAD

/** 遥控器路径 **/
// x : 2-左手 ; 0-右手
// y : 3-左手 ; 1-右手

#define CHASSIS_YAW_CHANNEL 0
#define CHASSIS_VX_CHANNEL 1
#define CHASSIS_LEG_CHANNEL 2

/** 变量约束 **/
#define MIN_L0 0.05f

#define MAX_CHASSIS_VX_SPEED 2.1f
#define MAX_WHEEL_TORQUE 3.0f
#define MIN_WHEEL_TORQUE (-3.0f)
#define MAX_JOINT_TORQUE 3.0f
#define MIN_JOINT_TORQUE (-3.0f)

/** 遥控器值映射 **/
#define RC_TO_VX  (MAX_CHASSIS_VX_SPEED/660)
#define MAX_CHASSIS_YAW_INCREMENT 0.01f
#define RC_TO_YAW_INCREMENT (MAX_CHASSIS_YAW_INCREMENT/660)


/****** PID参数 ******/

/** Wheel **/

//// 转向PID
//#define CHASSIS_TURN_POS_PID_P 5.0f
//#define CHASSIS_TURN_POS_PID_I 0.0f
//#define CHASSIS_TURN_POS_PID_D 0.0f
//#define CHASSIS_TURN_POS_PID_IOUT_LIMIT 0.0f
//#define CHASSIS_TURN_POS_PID_OUT_LIMIT 3.0f
//
//#define CHASSIS_TURN_SPEED_PID_P 10.0f
//#define CHASSIS_TURN_SPEED_PID_I 0.0f
//#define CHASSIS_TURN_SPEED_PID_D 0.0f
//#define CHASSIS_TURN_SPEED_PID_IOUT_LIMIT 0.0f
//#define CHASSIS_TURN_SPEED_PID_OUT_LIMIT 4.0f

// 转向PID
#define CHASSIS_TURN_POS_PID_P 10.0f
#define CHASSIS_TURN_POS_PID_I 0.0f
#define CHASSIS_TURN_POS_PID_D 0.0f
#define CHASSIS_TURN_POS_PID_IOUT_LIMIT 0.0f
#define CHASSIS_TURN_POS_PID_OUT_LIMIT 2.0f

#define CHASSIS_TURN_SPEED_PID_P 2.0f
#define CHASSIS_TURN_SPEED_PID_I 0.0f
#define CHASSIS_TURN_SPEED_PID_D 0.0f
#define CHASSIS_TURN_SPEED_PID_IOUT_LIMIT 0.0f
#define CHASSIS_TURN_SPEED_PID_OUT_LIMIT 3.0f

/** Joint **/

// 防劈叉PID
#define CHASSIS_LEG_COORDINATION_PID_P 50.0f // 20.0f 30.0f
#define CHASSIS_LEG_COORDINATION_PID_I 0.0f
#define CHASSIS_LEG_COORDINATION_PID_D 5.0f // 1.0f 5.0f
#define CHASSIS_LEG_COORDINATION_PID_IOUT_LIMIT 0.0f
#define CHASSIS_LEG_COORDINATION_PID_OUT_LIMIT 10.0f

// 腿长位置环PID
#define CHASSIS_LEG_L0_POS_PID_P 15.0f
#define CHASSIS_LEG_L0_POS_PID_I 0.0f
#define CHASSIS_LEG_L0_POS_PID_D 0.0f
#define CHASSIS_LEG_L0_POS_PID_IOUT_LIMIT 0.0f
#define CHASSIS_LEG_L0_POS_PID_OUT_LIMIT 2.0f

// 腿长速度环PID
#define CHASSIS_LEG_L0_SPEED_PID_P 30.0f
#define CHASSIS_LEG_L0_SPEED_PID_I 0.0f
#define CHASSIS_LEG_L0_SPEED_PID_D 0.0f
#define CHASSIS_LEG_L0_SPEED_PID_IOUT_LIMIT 0.0f
#define CHASSIS_LEG_L0_SPEED_PID_OUT_LIMIT 50.0f

// 离地后的腿长PID 暂时没用到
#define CHASSIS_OFFGROUND_LO_PID_P 0.0f
#define CHASSIS_OFFGROUND_L0_PID_I 0.0f
#define CHASSIS_OFFGROUND_L0_PID_D 0.0f
#define CHASSIS_OFFGROUND_L0_PID_IOUT_LIMIT 0.0f
#define CHASSIS_OFFGROUND_L0_PID_OUT_LIMIT 0.0f

// Roll补偿PID
#define CHASSIS_ROLL_PID_P 500.0f
#define CHASSIS_ROLL_PID_I 0.0f
#define CHASSIS_ROLL_PID_D 0.0f
#define CHASSIS_ROLL_PID_IOUT_LIMIT 0.0f
#define CHASSIS_ROLL_PID_OUT_LIMIT 50.0f



/** 底盘物理参数结构体 **/
typedef struct{
    float wheel_radius; // 驱动轮半径
    float body_weight; // 机体质量(有云台要算上云台)
    float wheel_weight; // 驱动轮重量(算上电机)
    float machine_limit_angle; // 驱动轮重量(算上电机)

    float l1, l2, l3, l4, l5; // 五连杆参数
} ChassisPhysicalConfig;

/** 底盘模式结构体 **/
typedef enum{
    CHASSIS_DISABLE = 1, // 失能模式
    CHASSIS_INIT, // 初始化模式
    CHASSIS_ENABLE, // 使能模式
    CHASSIS_SPIN, // 小陀螺
    CHASSIS_JUMP, // 跳跃模式
} ChassisCtrlMode;

/** 底盘状态结构体 -- 用于倒地自救 **/
typedef enum{
    CHASSIS_BODY_UNNORMAL,
    CHASSIS_BODY_NORMAL,
} ChassisBodyState;

typedef enum{
    CHASSIS_FALL_LEG_UNNORMAL,
    CHASSIS_FALL_LEG_NORMAL,
} ChassisFallLegState;

typedef enum{
    CHASSIS_COULD_NOT_RECOVER,
    CHASSIS_COULD_RECOVER,
} ChassisRecoverState;


typedef struct{
    float v_m_per_s; // 期望速度
    float yaw_rad;
    float roll_rad;
    float height_m; // 期望腿长
    float spin_speed;

} ChassisCtrlInfo;


/** 跳跃状态结构体 **/
typedef enum{
    NOT_READY,
    READY, // 第一阶段：收腿蓄力
    STRETCHING, // 第二阶段：伸腿蹬地
    SHRINKING, // 第三阶段：空中收腿
    LANDING, // 第四阶段：落地
} JumpState;

/** 传感器结构体 **/
typedef struct{
    // 欧拉角
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    float yaw_total_rad;


    //三轴角速度
    float pitch_gyro;
    float yaw_gyro;
    float roll_gyro;

    //三轴加速度
    float ax;
    float ay;
    float az;

    // 机体竖直向上的加速度
    float robot_az;

} IMUReference;


/** 状态变量结构体 **/
typedef struct{
    float theta; // 状态变量1
    float theta_dot; // 状态变量2
    float theta_last;
    float theta_dot_last;
    float theta_ddot;

    float x; // 状态变量3
    float x_dot; // 状态变量4
    float x_dot_last;
    float x_ddot;

    float phi; // 状态变量5
    float phi_dot; // 状态变量6
} StateVariable;

/** VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC **/

/** 正运动学解算  FK == Forward Kinematics(正运动学) **/
typedef struct{// 腿长
    float L0;
    float L0_last;
    float L0_dot;
    float L0_dot_last;
    float L0_ddot;
} FKL0;

typedef struct{// 五连杆中的角度
    float phi1;
    float phi2;
    float phi3;
    float phi4;

    float phi0; // 腿摆角
    float last_phi0;
    float d_phi0;// 摆角变化速度
    float last_d_phi0;
    float dd_phi0;
} FKPhi;

typedef struct{// 五连杆中的点坐标(Coordinates)
    float a_x, a_y;
    float b_x, b_y;
    float c_x, c_y;
    float d_x, d_y;
    float e_x, e_y;
} FKPointCoordinates;

typedef struct{
    FKL0 fk_L0;
    FKPhi fk_phi;
    FKPointCoordinates fk_point_coordinates;
    float d_alpha; // ?

/** 正动力学解算(Forward Dynamics)：从 末端力(F Tp) 到 末端执行器(T1 T4) **/
    union { // 自行学习联合体的特性: union
        float array[2][2];
        struct {
            float Tp_set_point;
            float Fy_set_point;
        } E;
    } Fxy_set_point;

    union {
        float array[2][2];
        struct {
            float x1_1;
            float x1_2;
            float x2_1;
            float x2_2;
        } E;
    } J_F_to_T;

    union {
        float array[2][1];
        struct {
            float T1_set_point;
            float T4_set_point;
        } E;
    } T1_T4_set_point;

} ForwardKinematics;


/** 逆动力学解算(Inverse Dynamics): 从 末端执行器(T1 T4) 到 末端力(T Tp) **/
typedef struct {
    union {
        float array[2][1];
        struct {
            float T1_fdb;
            float T4_fdb;
        } E;
    } T1_T4_fdb;

    union {
        float array[2][2];
        struct {
            float x1_1;
            float x1_2;
            float x2_1;
            float x2_2;
        } E;
    } J_T_to_F;

    union {
        float array[2][1];
        struct {
            float Tp_fdb;
            float Fy_fdb;
        } E;
    } Fxy_fdb;

    /** 逆运动学解算(Inverse Dynamics): 从 末端执行器(w1 w4) 到 末端编码(d_L0 d_phi0) **/
    union {
        float array[2][1];
        struct {
            float w1_fdb;// 关节电机反馈回来的角速度
            float w4_fdb;
        } E;
    } W_fdb;

    union {
        float array[2][2];
        struct {
            float x1_1;
            float x1_2;
            float x2_1;
            float x2_2;
        } E;
    } J_w_to_v;

    union {
        float array[2][1];
        struct {
            float d_L0_fdb; // 腿长变化速度
            float d_phi0_fdb; // 摆角(phi0)变化速度

            float last_d_L0_fdb;
            float dd_L0_fdb;
        } E;
    } V_fdb;

}InverseKinematics;

/** 腿部VMC结构体 **/
typedef struct{
    ForwardKinematics forward_kinematics;
    InverseKinematics inverse_kinematics;
} VMC;
/*****************************************************************************/



/** 腿部结构体 **/
typedef struct{

    ChassisCtrlInfo chassis_ctrl_info;

    /** 状态变量 **/
    StateVariable state_variable_feedback;  // 反馈状态变量
    StateVariable state_variable_set_point; // 期望状态变量
    StateVariable state_variable_error;     // 误差 = 反馈 - 期望
    StateVariable state_variable_wheel_out; // 各个状态变量通过lqr计算的关于轮毂的输出
    StateVariable state_variable_joint_out; // 各个状态变量通过lqr计算的关于关节的输出

    /** 腿部VMC **/
    VMC vmc;

    /** 腿长串级PID **/
    Pid leg_pos_pid; // 腿长位置环
    Pid leg_speed_pid; // 腿长速度环

    /** 离地后的腿长PID **/
    Pid offground_leg_pid; // 离地后的腿长pid  使腿尽量接近地面，增加缓冲

    float wheel_torque; // 轮毂力矩
    float joint_F_torque; // 关节力矩
    float joint_B_torque;

    /** 竖直方向支持力 **/
    float Fn;

    /** 滤波器 **/

    // theta_dot的微分
    LowPassFilter theta_dot_lpf;

} Leg;

/** 底盘结构体 **/
typedef struct{

    /** 传感器 **/
    IMUReference imu_reference;

    /** 遥控器信息 **/
    ChassisCtrlMode chassis_ctrl_mode;
    ChassisCtrlMode chassis_last_ctrl_mode;
    ChassisCtrlInfo chassis_ctrl_info;

    /** 腿部 **/
    Leg leg_L;
    Leg leg_R;


    /****** PID ******/
    /** Wheel **/

    // 转向PID
    Pid chassis_turn_pos_pid;
    Pid chassis_turn_speed_pid;

    float wheel_turn_torque;          // 转向力矩

    /** Joint **/

    // 防劈叉PID
    float phi0_error;
    float last_phi0_error;
    float d_phi0_error;
    Pid chassis_leg_coordination_pid;
    float steer_compensatory_torque;  // 防劈叉力矩

    // Roll补偿PID
    Pid chassis_roll_pid;

    // 机体状态
    ChassisBodyState chassis_body_state;

    ChassisFallLegState chassis_fall_leg_state;

    ChassisRecoverState chassis_recover_state;

    // flag
    bool init_flag;            // 底盘初始化完成标志位
    bool chassis_recover_finish;

} Chassis;


extern float Kd;
extern float vel;

extern Chassis chassis;
extern ChassisPhysicalConfig chassis_physical_config;


/*******************************************************************************
 *                                  板间通信                                    *
 *******************************************************************************/
extern Gimbal_Unpack_Data gimbal_unpack_data;

#endif
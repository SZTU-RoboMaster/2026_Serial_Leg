#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include <stdbool.h>
#include "pid.h"
#include "can_device.h"
#include "user_lib.h"
#include "DJI_motor.h"
#include "board_communication_task.h"
#include "low_pass_filter.h"

/*******************************************************************************
 *                                 遥控                                      *
 *******************************************************************************/

/* kss：遥控器路径
                x : 2-左手 ; 0-右手
                y : 3-左手 ; 1-右手 */

/* =========================== 遥控器通道 ============================= */

    /* =========================== yaw通道 ============================= */

        #define CHASSIS_YAW_CHANNEL 0

    /* =========================== v通道 ============================= */

        #define CHASSIS_VX_CHANNEL 1

    /* =========================== L0通道 ============================= */

        #define CHASSIS_LEG_CHANNEL 2


/* =========================== 遥控器值映射 ============================= */

    /* =========================== v映射 ============================= */

            #define RC_TO_VX  (MAX_CHASSIS_VX_SPEED/660)

    /* =========================== yaw映射 ============================= */

            #define MAX_CHASSIS_YAW_INCREMENT 0.01f

    /* =========================== L0映射 ============================= */

            #define RC_TO_YAW_INCREMENT (MAX_CHASSIS_YAW_INCREMENT/660)

/* =========================== 遥控命令 ============================= */

    typedef struct
    {
        /* =========================== 期望速度 ============================= */

        float v_m_per_s;

        /* =========================== 期望yaw ============================= */

        float yaw_rad;

        /* =========================== 期望腿长 ============================= */

        float target_length;

    } ChassisCtrlInfo;

/*****************************************************************************
 *                                 底盘                                      *
 *****************************************************************************/

/* =========================== 单位转换 ============================= */

    /* =========================== rpm-rad/s ============================= */

        #define RPM_TO_RAD_PER_S (PI/ 30)

    /* =========================== rpm-m/s ============================= */

        #define RPM_TO_M_PER_S (PI * chassis_physical_config.wheel_radius) / 30

/* =========================== 底盘运行周期（ms） ============================= */

    #define CHASSIS_PERIOD 1

/* =========================== phi偏置 ============================= */

    /* iss：按零点调 */

    #define PHI1_OFFSET 2.20f
    #define PHI4_OFFSET 1.10f

/* =========================== 目标状态 ============================= */

    /* =========================== φ ============================= */

        #define PHI_OFFSET 0.0f

    /* =========================== θ ============================= */

        #define THETA_OFFSET 0.0f

/* =========================== 小陀螺旋转速度 ============================= */

    #define SPIN_SPEED 5.0f

/* =========================== 腿摆角 用于倒地自救 ============================= */

    /* iss：要调 */

    #define LEG_NORMAL_RAD 30.0f * DEGREE_TO_RAD

/* =========================== 约束 ============================= */

    /* =========================== L0约束 ============================= */

        #define MIN_L0 0.16f
        #define MID_L0 0.24f
        #define MAX_L0 0.30f

    /* =========================== v约束 ============================= */

        #define MAX_CHASSIS_VX_SPEED 2.1f

    /* =========================== 轮毂力矩约束 ============================= */

        #define MAX_WHEEL_TORQUE 3.0f
        #define MIN_WHEEL_TORQUE (-3.0f)

    /* =========================== 关节力矩约束 ============================= */

        #define MAX_JOINT_TORQUE 40.0f
        #define MIN_JOINT_TORQUE (-40.0f)

/* =========================== PID参数 ============================= */

    /* iss：要调 */

    /* =========================== 轮毂 ============================= */

        /* =========================== 转向PID ============================= */

            /* =========================== 转向位置环PID ============================= */

                #define CHASSIS_TURN_POS_PID_P 1.0f
                #define CHASSIS_TURN_POS_PID_I 0.0f
                #define CHASSIS_TURN_POS_PID_D 0.0f
                #define CHASSIS_TURN_POS_PID_IOUT_LIMIT 0.0f
                #define CHASSIS_TURN_POS_PID_OUT_LIMIT 2.0f

            /* =========================== 转向速度环PID ============================= */

                #define CHASSIS_TURN_SPEED_PID_P 1.0f
                #define CHASSIS_TURN_SPEED_PID_I 0.0f
                #define CHASSIS_TURN_SPEED_PID_D 0.0f
                #define CHASSIS_TURN_SPEED_PID_IOUT_LIMIT 0.0f
                #define CHASSIS_TURN_SPEED_PID_OUT_LIMIT 3.0f

    /* =========================== 关节 ============================= */

        /* =========================== 防劈叉PID ============================= */

        #define CHASSIS_LEG_COORDINATION_PID_P 0.0f // 1
        #define CHASSIS_LEG_COORDINATION_PID_I 0.0f
        #define CHASSIS_LEG_COORDINATION_PID_D 0.0f
        #define CHASSIS_LEG_COORDINATION_PID_IOUT_LIMIT 0.0f
        #define CHASSIS_LEG_COORDINATION_PID_OUT_LIMIT 10.0f

        /* =========================== 腿长pid============================= */

            /* =========================== 腿长位置环PID ============================= */

                #define CHASSIS_LEG_L0_POS_PID_P 6.0f
                #define CHASSIS_LEG_L0_POS_PID_I 0.0f
                #define CHASSIS_LEG_L0_POS_PID_D 0.0f
                #define CHASSIS_LEG_L0_POS_PID_IOUT_LIMIT 0.0f
                #define CHASSIS_LEG_L0_POS_PID_OUT_LIMIT 2.0f

            /* =========================== 腿长速度环PID ============================= */

                #define CHASSIS_LEG_L0_SPEED_PID_P 10.0f
                #define CHASSIS_LEG_L0_SPEED_PID_I 0.0f
                #define CHASSIS_LEG_L0_SPEED_PID_D 0.0f
                #define CHASSIS_LEG_L0_SPEED_PID_IOUT_LIMIT 0.0f
                #define CHASSIS_LEG_L0_SPEED_PID_OUT_LIMIT 50.0f

        /* =========================== Roll补偿PID ============================= */

            #define CHASSIS_ROLL_PID_P 200.0f
            #define CHASSIS_ROLL_PID_I 0.0f
            #define CHASSIS_ROLL_PID_D 0.0f
            #define CHASSIS_ROLL_PID_IOUT_LIMIT 0.0f
            #define CHASSIS_ROLL_PID_OUT_LIMIT 50.0f

/* =========================== 底盘物理参数结构体 ============================= */

    typedef struct
    {
        float wheel_radius;
        float body_weight;
        float wheel_weight;
        float l1, l2, l3, l4, l5;
    } ChassisPhysicalConfig;

/* =========================== 底盘模式 ============================= */

    typedef enum
    {
            CHASSIS_DISABLE = 1,
            CHASSIS_INIT,
            CHASSIS_ENABLE,
            CHASSIS_SPIN,
    } ChassisCtrlMode;

/* =========================== 底盘运行状态 用于倒地自救 ============================= */

    typedef enum
    {
        CHASSIS_UNNORMAL,
        CHASSIS_NORMAL,
    } ChassisState;

/* =========================== 传感器信息 ============================= */

    typedef struct
    {
        /* =========================== roll ============================= */

            float roll_rad;

        /* =========================== pitch ============================= */

            float pitch_rad;

        /* =========================== yaw ============================= */

            /* =========================== 达妙IMU读到的yaw ============================= */

                float yaw_rad;

            /* =========================== 上次yaw ============================= */

                float yaw_last_rad;

            /* =========================== 圈数 ============================= */

                float yaw_round_count;

            /* =========================== 实际yaw ============================= */

                float yaw_total_rad;

        /* =========================== 三轴角速度 ============================= */

            float pitch_gyro;
            float yaw_gyro;
            float roll_gyro;

        /* =========================== 三轴加速度 ============================= */

            float ax;
            float ay;
            float az;

        /* =========================== 机体在世界坐标系移动的加速度 ============================= */

            float robot_ax;
            float robot_az;

    } IMUReference;

/* =========================== 状态变量 =========================== */

    typedef struct
    {

        /* =========================== θ相关 =========================== */

            float theta;
            float theta_dot;

            float theta_dot_last;
            float theta_ddot;

        /* =========================== x相关 =========================== */

            float x;
            float x_dot;

            float x_dot_last;
            float x_ddot;

        /* =========================== φ相关 =========================== */

            float phi;
            float phi_dot;

    } StateVariable;

/* =========================== 五连杆中的L0 =========================== */

    typedef struct
    {
        float L0;
        float L0_dot;
        float L0_dot_last;
        float L0_ddot;
    } FKL0;

/* =========================== 五连杆中的角度 =========================== */

    typedef struct
    {
        float phi1;
        float phi2;
        float phi3;
        float phi4;

        float phi1_dot;
        float phi4_dot;

        float phi0;
        float d_phi0;
        float last_d_phi0;
        float dd_phi0;

    } FKPhi;

/* =========================== 五连杆中的坐标 =========================== */

    typedef struct
    {
        float a_x, a_y;
        float b_x, b_y;
        float c_x, c_y;
        float d_x, d_y;
        float e_x, e_y;
    } FKPointCoordinates;

/* =========================== VMC正解算 =========================== */

    typedef struct
    {
        /* =========================== 五连杆中的L0 =========================== */

            FKL0 fk_L0;

        /* =========================== 五连杆中的角度 =========================== */

            FKPhi fk_phi;

        /* =========================== 五连杆中的坐标 =========================== */

            FKPointCoordinates fk_point_coordinates;

        /* =========================== Tp F =========================== */

            union
            {
                float array[2][2];
                struct
                {
                    float Tp_set_point;
                    float Fy_set_point;
                } E;
            } Fxy_set_point;

        /* =========================== 转换 =========================== */

            union {
                float array[2][2];
                struct {
                    float x1_1;
                    float x1_2;
                    float x2_1;
                    float x2_2;
                } E;
            } J_F_to_T;

        /* =========================== T1,T4 =========================== */

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

} InverseKinematics;

/* =========================== VMC =========================== */

    typedef struct
    {
        /* =========================== vmc正解算 =========================== */

            ForwardKinematics forward_kinematics;

        /* =========================== vmc逆解算 =========================== */

            InverseKinematics inverse_kinematics;

    } VMC;

/* =========================== 腿部 =========================== */

    typedef struct
    {
        /* =========================== 遥控命令 ============================= */

            ChassisCtrlInfo chassis_ctrl_info;

        /* =========================== 状态变量 =========================== */

            /* =========================== 反馈状态变量 =========================== */

                StateVariable state_variable_feedback;
                StateVariable state_variable_error;
                StateVariable state_variable_wheel_out;
                StateVariable state_variable_joint_out;

        /* =========================== 腿部VMC =========================== */

            VMC vmc;

        /* =========================== 腿长pid =========================== */

            Pid leg_pos_pid;
            Pid leg_speed_pid;
            float leg_change_torque;

        /* =========================== 轮毂力矩 =========================== */

            float wheel_torque;

        /* =========================== 关节力矩 速度 kd =========================== */

            float joint_F_torque;
            float joint_B_torque;
            float speed;
            float kd;

        /* =========================== 竖直方向支持力 =========================== */

            float Fn;

        /* =========================== 滤波器 =========================== */

            LowPassFilter theta_ddot;
            LowPassFilter x_ddot;

    } Leg;

/* =========================== 底盘结构体 ============================= */

    typedef struct {

        /* =========================== 传感器信息 ============================= */

            IMUReference imu_reference;

        /* =========================== 遥控器信息 ============================= */

            /* =========================== 底盘模式 ============================= */

                ChassisCtrlMode chassis_ctrl_mode;

            /* =========================== 上次底盘模式 ============================= */

                ChassisCtrlMode chassis_last_ctrl_mode;

            /* =========================== 遥控命令 ============================= */

                ChassisCtrlInfo chassis_ctrl_info;

        /* =========================== 腿部 ============================= */

            Leg leg_L;
            Leg leg_R;

        /* =========================== PID ============================= */

            /* =========================== 轮毂 ============================= */

                /* =========================== 转向PID ============================= */

                    /* =========================== 转向位置环PID ============================= */

                            Pid chassis_turn_pos_pid;

                    /* =========================== 转向速度环PID ============================= */

                            Pid chassis_turn_speed_pid;

                    /* =========================== 转向力矩 ============================= */

                            float wheel_turn_torque;

            /* =========================== 关节 ============================= */

                /* =========================== 防劈叉PID ============================= */

                    /* =========================== φ差 ============================= */

                        float phi0_error;

                    /* =========================== 上次φ差 ============================= */

                        float last_phi0_error;

                    /* =========================== φ差' ============================= */

                        float d_phi0_error;

                    /* =========================== 防劈叉PID ============================= */

                        Pid chassis_leg_coordination_pid;

                    /* =========================== 防劈叉补偿力矩 ============================= */

                        float steer_compensatory_torque;

            /* =========================== Roll补偿PID ============================= */

                /* =========================== Roll补偿PID ============================= */

                    Pid chassis_roll_pid;

                /* =========================== Roll补偿力矩 ============================= */

                    float roll_compensatory_torque;

        /* =========================== 机体状态 用于倒地自救 ============================= */

            ChassisState chassis_fall_leg_state;

        /* =========================== 标志位 =========================== */

            /* =========================== 底盘初始化标志位 完成1 =========================== */

                bool init_flag;

            /* =========================== 倒地自救标志位 完成1 =========================== */

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
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
 *                                    ����                                     *
 *******************************************************************************/

/** �궨�� **/
// phiƫ��
#define PHI1_OFFSET (2.2f)
#define PHI4_OFFSET (1.0f)

#define THETA_OFFSET 0.05f // 0.1rad
#define PHI_OFFSET -0.05f
// ������������
#define CHASSIS_PERIOD 1 // ms ����Ƶ�� ���˹���

#define RPM_TO_RAD_PER_S (PI/ 30) // (rad/s) = (rpm) * (pi/30)
#define RPM_TO_M_PER_S (PI * chassis_physical_config.wheel_radius) / 30

// ��ת�ٶ�
#define SPIN_SPEED 5.0f

/** ң����·�� **/
// x : 2-���� ; 0-����
// y : 3-���� ; 1-����

#define CHASSIS_YAW_CHANNEL 0
#define CHASSIS_VX_CHANNEL 1
#define CHASSIS_LEG_CHANNEL 2

/** ����Լ�� **/
#define MIN_L0 0.16f
#define MID_L0 0.24f
#define MAX_L0 0.30f

#define MAX_CHASSIS_VX_SPEED 2.1f
#define MAX_WHEEL_TORQUE 2.0f
#define MIN_WHEEL_TORQUE (-2.0f)
//#define MAX_JOINT_TORQUE 40.0f // 1.0f 40.0f
//#define MIN_JOINT_TORQUE (-40.0f) // -1.0f -40.0f
#define MAX_JOINT_TORQUE 15.0f
#define MIN_JOINT_TORQUE (-15.0f)

// ����/ʧ��ʶ����ֵ���ⲿ��Ŀ˼·��pitch/roll/theta��һ����Խ�缴�����Ծ�
#define CHASSIS_FALL_THETA_ENTER (45.0f * DEGREE_TO_RAD)
#define CHASSIS_FALL_ATTITUDE_ENTER (45.0f * DEGREE_TO_RAD)
#define CHASSIS_FALL_L0_ENTER 0.28f
// �����ԾȽ׶�Ŀ�꣺��ѡstage0�м���̬��stage1������theta����
#define CHASSIS_SELFHELP_THETA_STAGE0 (0.36f * PI)
#define CHASSIS_SELFHELP_L0_STAGE0 MAX_L0
#define CHASSIS_SELFHELP_THETA_STAGE1 0.0f
#define CHASSIS_SELFHELP_L0_STAGE1 MIN_L0
#define CHASSIS_SELFHELP_STAGE0_L0_MARGIN 0.10f
#define CHASSIS_SELFHELP_THETA_EXIT (15.0f * DEGREE_TO_RAD)
#define CHASSIS_SELFHELP_L0_EXIT 0.05f
#define CHASSIS_SELFHELP_EXIT_CONFIRM_COUNT 100U
#define CHASSIS_SELFHELP_GRAVITY_COMP_FF 30.0f
// �����Ծ�theta��λPID�������ΪVMC��Tp_set_point�����ڰ�theta����Ŀ�긽��
#define CHASSIS_SELFHELP_PHI0_PID_P 15.0f
#define CHASSIS_SELFHELP_PHI0_PID_I 0.0f
#define CHASSIS_SELFHELP_PHI0_PID_D 3.0f
#define CHASSIS_SELFHELP_PHI0_PID_IOUT_LIMIT 0.0f
#define CHASSIS_SELFHELP_PHI0_PID_OUT_LIMIT 8.0f

/** ң����ֵӳ�� **/
#define RC_TO_VX  (MAX_CHASSIS_VX_SPEED/660)
#define MAX_CHASSIS_YAW_INCREMENT 0.01f
#define RC_TO_YAW_INCREMENT (MAX_CHASSIS_YAW_INCREMENT/660)
// 临时：遥控 yaw 通道直接映射为转向力矩，先绕开 yaw 位置环
#define CHASSIS_RC_YAW_TO_TORQUE_K 0.004f

/****** PID���� ******/

/** Wheel **/

// ת��PID
#define CHASSIS_TURN_POS_PID_P 1.0f
#define CHASSIS_TURN_POS_PID_I 0.0f
#define CHASSIS_TURN_POS_PID_D 0.0f
#define CHASSIS_TURN_POS_PID_IOUT_LIMIT 0.0f
#define CHASSIS_TURN_POS_PID_OUT_LIMIT 2.0f

#define CHASSIS_TURN_SPEED_PID_P 1.0f
#define CHASSIS_TURN_SPEED_PID_I 0.0f
#define CHASSIS_TURN_SPEED_PID_D 0.0f
#define CHASSIS_TURN_SPEED_PID_IOUT_LIMIT 0.0f
#define CHASSIS_TURN_SPEED_PID_OUT_LIMIT 3.0f

/** Joint **/
// ������PID
#define CHASSIS_LEG_COORDINATION_PID_P 5.0f // 1
#define CHASSIS_LEG_COORDINATION_PID_I 0.0f
#define CHASSIS_LEG_COORDINATION_PID_D 0.0f
#define CHASSIS_LEG_COORDINATION_PID_IOUT_LIMIT 0.0f
#define CHASSIS_LEG_COORDINATION_PID_OUT_LIMIT 10.0f

// �ȳ�λ�û�PID
#define CHASSIS_LEG_L0_POS_PID_P 20.0f
#define CHASSIS_LEG_L0_POS_PID_I 0.0f
#define CHASSIS_LEG_L0_POS_PID_D 0.0f
#define CHASSIS_LEG_L0_POS_PID_IOUT_LIMIT 0.0f
#define CHASSIS_LEG_L0_POS_PID_OUT_LIMIT 2.0f

// �ȳ��ٶȻ�PID
#define CHASSIS_LEG_L0_SPEED_PID_P 25.0f
#define CHASSIS_LEG_L0_SPEED_PID_I 0.0f
#define CHASSIS_LEG_L0_SPEED_PID_D 0.0f
#define CHASSIS_LEG_L0_SPEED_PID_IOUT_LIMIT 0.0f
#define CHASSIS_LEG_L0_SPEED_PID_OUT_LIMIT 50.0f

// Roll����PID
#define CHASSIS_ROLL_PID_P 200.0f // 200
#define CHASSIS_ROLL_PID_I 0.0f
#define CHASSIS_ROLL_PID_D 0.0f
#define CHASSIS_ROLL_PID_IOUT_LIMIT 0.0f
#define CHASSIS_ROLL_PID_OUT_LIMIT 50.0f


/** �������������ṹ�� **/
typedef struct {
    float wheel_radius; // �����ְ뾶
    float body_weight; // ��������(����̨Ҫ������̨)
    float wheel_weight; // ����������(���ϵ��)
    float machine_limit_angle; // ����������(���ϵ��)

    float l1, l2, l3, l4, l5; // �����˲���
} ChassisPhysicalConfig;

/** ����ģʽ�ṹ�� **/
typedef enum {
    CHASSIS_DISABLE = 1, // ʧ��ģʽ
    CHASSIS_INIT, // ��ʼ��ģʽ
    CHASSIS_ENABLE, // ʹ��ģʽ
    CHASSIS_SPIN, // С����
    CHASSIS_JUMP, // ��Ծģʽ
} ChassisCtrlMode;

/** ��������״̬ **/
typedef enum {
    CHASSIS_FALL = 1, // ����״̬����Ҫ�����Ծ�
    CHASSIS_STAND_UP, // �Ծ���ɺ������״̬��ʹ�����LQR�ص�ƽ���
    CHASSIS_NORMAL, // ��������״̬
} ChassisState;


typedef struct {
    float v_m_per_s; // �����ٶ�
    float yaw_rad;
    float roll_rad;
    float spin_speed;

    float target_length; // �����ȳ�

} ChassisCtrlInfo;


/** �������ṹ�� **/
typedef struct {
    // ŷ����
    float roll_rad;
    float pitch_rad;

    float yaw_rad;
    float yaw_last_rad;
    int16_t yaw_round_count;
    float yaw_total_rad;

    //������ٶ�
    float pitch_gyro;
    float yaw_gyro;
    float roll_gyro;

    //������ٶ�
    float ax;
    float ay;
    float az;

    // ��������������ϵ��ֱ���ϵļ��ٶ�
    float robot_az;

    // ��������������ϵ�ƶ��ļ��ٶ�
    float robot_ax;

} IMUReference;


/** ״̬�����ṹ�� **/
typedef struct {
    float theta; // ״̬����1
    float theta_dot; // ״̬����2
    float theta_dot_last;
    float theta_ddot;

    float x; // ״̬����3
    float x_dot; // ״̬����4
    float x_dot_last;
    float x_ddot;

    float phi; // ״̬����5
    float phi_dot; // ״̬����6
} StateVariable;

/** VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC VMC **/

/** ���˶�ѧ����  FK == Forward Kinematics(���˶�ѧ) **/
typedef struct {// �ȳ�
    float L0;
    float L0_dot;
    float L0_dot_last;
    float L0_ddot;
} FKL0;

typedef struct {// �������еĽǶ�
    float phi1;
    float phi2;
    float phi3;
    float phi4;

    float phi1_dot;
    float phi4_dot;
    float phi3_dot;

    float phi0; // �Ȱڽ�
    float d_phi0;// �ڽǱ仯�ٶ�
    float last_d_phi0;
    float dd_phi0;

} FKPhi;

typedef struct {// �������еĵ�����(Coordinates)
    float a_x, a_y;
    float b_x, b_y;
    float c_x, c_y;
    float d_x, d_y;
    float e_x, e_y;
} FKPointCoordinates;

typedef struct {
    FKL0 fk_L0;
    FKPhi fk_phi;
    FKPointCoordinates fk_point_coordinates;

/** ������ѧ����(Forward Dynamics)���� ĩ����(F Tp) �� ĩ��ִ����(T1 T4) **/
    union { // ����ѧϰ�����������: union
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


/** �涯��ѧ����(Inverse Dynamics): �� ĩ��ִ����(T1 T4) �� ĩ����(T Tp) **/
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
            float w1_fdb;// �ؽڵ�����������Ľ��ٶ�
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
            float d_L0_fdb; // �ȳ��仯�ٶ�
            float d_phi0_fdb; // �ڽ�(phi0)�仯�ٶ�

            float last_d_L0_fdb;
            float dd_L0_fdb;
        } E;
    } V_fdb;

} InverseKinematics;


/** �Ȳ�VMC�ṹ�� **/
typedef struct {
    ForwardKinematics forward_kinematics;
    InverseKinematics inverse_kinematics;
} VMC;
/*****************************************************************************/



/** �Ȳ��ṹ�� **/
typedef struct {

    ChassisCtrlInfo chassis_ctrl_info;

    /** ״̬���� **/
    StateVariable state_variable_feedback;  // ����״̬����
    StateVariable state_variable_ref;       // ����״̬����
    StateVariable state_variable_error;     // ��� = ���� - ����
    StateVariable state_variable_wheel_out; // ����״̬����ͨ��lqr����Ĺ�����챵����
    StateVariable state_variable_joint_out; // ����״̬����ͨ��lqr����Ĺ��ڹؽڵ����

    /** �Ȳ�VMC **/
    VMC vmc;

    /** �ȳ�����PID **/
    Pid leg_pos_pid; // �ȳ�λ�û�
    Pid leg_speed_pid; // �ȳ��ٶȻ�

    float wheel_torque; // �������
    float joint_F_torque; // �ؽ�����
    float joint_B_torque;

    /** ��ֱ����֧���� **/
    float Fn;

    /** �˲��� **/
} Leg;

/** ���̽ṹ�� **/
typedef struct {

    /** ������ **/
    IMUReference imu_reference;

    /** ң������Ϣ **/
    ChassisCtrlMode chassis_ctrl_mode;
    ChassisCtrlMode chassis_last_ctrl_mode;
    ChassisState chassis_state; // ENABLE������ʹ�õĵ�������״̬�����������������к͵����Ծ�
    bool selfhelp_done;          // �����Ծ��Ƿ���λ
    ChassisCtrlInfo chassis_ctrl_info;

    /** �Ȳ� **/
    Leg leg_L;
    Leg leg_R;


    /****** PID ******/
    /** Wheel **/

    // ת��PID
    Pid chassis_turn_pos_pid;
    Pid chassis_turn_speed_pid;

    float wheel_turn_torque;          // ת������

    /** Joint **/

    // ������PID
    float phi0_error;
    float last_phi0_error;
    float d_phi0_error;
    Pid chassis_leg_coordination_pid;
    float steer_compensatory_torque;  // ����������

    // Roll����PID
    Pid chassis_roll_pid;
    float roll_compensatory_torque; // Roll��������

    // �����Ծ�theta��λPID���ֱ����������theta�ص�Ŀ�긽��
    Pid chassis_selfhelp_phi0_pid_L;
    Pid chassis_selfhelp_phi0_pid_R;

} Chassis;

extern float Kd;
extern float vel;

extern Chassis chassis;
extern ChassisPhysicalConfig chassis_physical_config;


/*******************************************************************************
 *                                  ���ͨ��                                    *
 *******************************************************************************/
extern Gimbal_Unpack_Data gimbal_unpack_data;

#endif
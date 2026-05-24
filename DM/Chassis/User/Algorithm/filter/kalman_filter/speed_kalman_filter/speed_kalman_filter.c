#include "speed_kalman_filter.h"
#include "wheel.h"
#include "robot_def.h"
#include "vofa.h"
#include <math.h>

/** 速度-加速度融合 **/
KalmanFilter_t Speed_EstimateKF;       // 卡尔曼滤波器结构体

float vel_acc[2]; // 轮毂速度与加速度融合后的结果

float Speed_EstimateKF_F[4] = {1.0f, CHASSIS_PERIOD * 0.001f,
                               0.0f, 1.0f};       // 状态转移矩阵，控制周期为0.001s

float Speed_EstimateKF_P[4] = {1.0f, 0.0f,
                               0.0f, 1.0f};    // 后验估计协方差初始值

float Speed_EstimateKF_Q[4] = {VEL_PROCESS_NOISE, 0.0f,
                               0.0f, ACC_PROCESS_NOISE};    // Q矩阵初始值、先验估计值方差噪声

float Speed_EstimateKF_R[4] = {VEL_MEASURE_NOISE, 0.0f,
                               0.0f, ACC_MEASURE_NOISE};    // 测量噪声方差

const float Speed_EstimateKF_H[4] = {1.0f, 0.0f,
                                     0.0f, 1.0f};    // 设置矩阵H为常量


/*******************************************************************************
 *                                  速度融合                                    *
 *******************************************************************************/
void Speed_EstimateKF_Init(KalmanFilter_t *Speed_EstimateKF)//初始化卡尔曼结构体，并把该开头定义的矩阵复制到结构体中的矩阵
{
    Kalman_Filter_Init(Speed_EstimateKF, 2, 0, 2);    // 状态向量2维 没有控制量 测量向量2维

    memcpy(Speed_EstimateKF->F_data, Speed_EstimateKF_F, sizeof(Speed_EstimateKF_F));
    memcpy(Speed_EstimateKF->P_data, Speed_EstimateKF_P, sizeof(Speed_EstimateKF_P));
    memcpy(Speed_EstimateKF->Q_data, Speed_EstimateKF_Q, sizeof(Speed_EstimateKF_Q));
    memcpy(Speed_EstimateKF->R_data, Speed_EstimateKF_R, sizeof(Speed_EstimateKF_R));
    memcpy(Speed_EstimateKF->H_data, Speed_EstimateKF_H, sizeof(Speed_EstimateKF_H));

}

static void Speed_EstimateKF_Update(KalmanFilter_t *Speed_EstimateKF, float vel, float acc, float dt) {
    // 卡尔曼滤波器测量值更新
    Speed_EstimateKF->MeasuredVector[0] = vel; // 测量速度
    Speed_EstimateKF->MeasuredVector[1] = acc; // 测量加速度
    Speed_EstimateKF->F_data[1] = dt;

    // 卡尔曼滤波器更新函数
    Kalman_Filter_Update(Speed_EstimateKF);

    // 提取估计值
    for (uint8_t i = 0; i < 2; i++) {
        vel_acc[i] = Speed_EstimateKF->FilteredValue[i];
    }
}

float wheel_w_l, wheel_w_r; // 左右驱动轮转子相对大地的角速度，按外部项目phi3_w方法补偿
float v_l, v_r; // 左右驱动轮转子相对大地的线速度
float v_lb, v_rb; // 左右驱动轮计算出的机体速度
float aver_v; // 机体速度平均值
float phi3_dot_l;
float phi3_dot_r;

void speed_calc(float dt) {

    phi3_dot_l = chassis.leg_L.vmc.forward_kinematics.fk_phi.phi3_dot;
    phi3_dot_r = chassis.leg_R.vmc.forward_kinematics.fk_phi.phi3_dot;

    // 按外部项目思路：wheel_w = wheel_motor_w + phi3_w + pitch_w。
    wheel_w_l =  ( -get_wheel_motors()->speed_aps ) * DEGREE_TO_RAD / RATIO
               +  phi3_dot_l
               +  chassis.imu_reference.pitch_gyro;

    v_l = wheel_w_l * chassis_physical_config.wheel_radius;

    wheel_w_r = ( get_wheel_motors() + 1 )->speed_aps * DEGREE_TO_RAD / RATIO
               +  phi3_dot_r
               +  chassis.imu_reference.pitch_gyro;

    v_r = wheel_w_r * chassis_physical_config.wheel_radius;

    v_lb = v_l + chassis.leg_L.vmc.forward_kinematics.fk_L0.L0_dot * sinf(chassis.leg_L.state_variable_feedback.theta) +
           chassis.leg_L.vmc.forward_kinematics.fk_L0.L0 * chassis.leg_L.state_variable_feedback.theta_dot *
           cosf(chassis.leg_L.state_variable_feedback.theta);

    v_rb = v_r + chassis.leg_R.vmc.forward_kinematics.fk_L0.L0_dot * sinf(chassis.leg_R.state_variable_feedback.theta) +
           chassis.leg_R.vmc.forward_kinematics.fk_L0.L0 * chassis.leg_R.state_variable_feedback.theta_dot *
           cosf(chassis.leg_R.state_variable_feedback.theta);

    aver_v = (v_lb + v_rb) / 2;

    Speed_EstimateKF_Update(&Speed_EstimateKF, aver_v, chassis.imu_reference.robot_ax, dt); // 不断更新卡尔曼滤波中的各项参数

    /********************* x x_dot ***********************/
    float feedback_vel = vel_acc[0];
    float ref_vel = chassis.chassis_ctrl_info.v_m_per_s;

    chassis.leg_L.state_variable_feedback.x_dot_last = chassis.leg_L.state_variable_feedback.x_dot;
    chassis.leg_R.state_variable_feedback.x_dot_last = chassis.leg_R.state_variable_feedback.x_dot;

    if (chassis.chassis_ctrl_mode == CHASSIS_DISABLE || chassis.chassis_state == CHASSIS_FALL)
    {
        chassis.leg_L.state_variable_feedback.x = 0.0f;
        chassis.leg_R.state_variable_feedback.x = 0.0f;
        chassis.leg_L.state_variable_feedback.x_dot = 0.0f;
        chassis.leg_R.state_variable_feedback.x_dot = 0.0f;

        chassis.leg_L.state_variable_ref.x = 0.0f;
        chassis.leg_R.state_variable_ref.x = 0.0f;
        chassis.leg_L.state_variable_ref.x_dot = 0.0f;
        chassis.leg_R.state_variable_ref.x_dot = 0.0f;
    } else
    {
        chassis.leg_L.state_variable_feedback.x_dot = feedback_vel;
        chassis.leg_R.state_variable_feedback.x_dot = feedback_vel;
        chassis.leg_L.state_variable_feedback.x += feedback_vel * dt;
        chassis.leg_R.state_variable_feedback.x += feedback_vel * dt;

        chassis.leg_L.state_variable_ref.x_dot = ref_vel;
        chassis.leg_R.state_variable_ref.x_dot = ref_vel;
        chassis.leg_L.state_variable_ref.x += ref_vel * dt;
        chassis.leg_R.state_variable_ref.x += ref_vel * dt;
    }

    chassis.leg_L.state_variable_error.x =
            chassis.leg_L.state_variable_feedback.x - chassis.leg_L.state_variable_ref.x;
    chassis.leg_R.state_variable_error.x =
            chassis.leg_R.state_variable_feedback.x - chassis.leg_R.state_variable_ref.x;
    chassis.leg_L.state_variable_error.x_dot =
            chassis.leg_L.state_variable_feedback.x_dot - chassis.leg_L.state_variable_ref.x_dot;
    chassis.leg_R.state_variable_error.x_dot =
            chassis.leg_R.state_variable_feedback.x_dot - chassis.leg_R.state_variable_ref.x_dot;

    // 4.1 x_ddot
    chassis.leg_L.state_variable_feedback.x_ddot =
            (chassis.leg_L.state_variable_feedback.x_dot - chassis.leg_L.state_variable_feedback.x_dot_last) /
                    dt;

    chassis.leg_R.state_variable_feedback.x_ddot =
            (chassis.leg_R.state_variable_feedback.x_dot - chassis.leg_R.state_variable_feedback.x_dot_last) /
                    dt;
}
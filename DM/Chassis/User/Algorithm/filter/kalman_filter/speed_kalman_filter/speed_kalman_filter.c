#include "speed_kalman_filter.h"
#include "wheel.h"
#include "robot_def.h"
#include "vofa.h"

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

static void Speed_EstimateKF_Update(KalmanFilter_t *Speed_EstimateKF, float acc, float vel) {
    //卡尔曼滤波器测量值更新
    Speed_EstimateKF->MeasuredVector[0] = vel;//测量速度
    Speed_EstimateKF->MeasuredVector[1] = acc;//测量加速度

    //卡尔曼滤波器更新函数
    Kalman_Filter_Update(Speed_EstimateKF);

    // 提取估计值
    for (uint8_t i = 0; i < 2; i++) {
        vel_acc[i] = Speed_EstimateKF->FilteredValue[i];
    }
}

void speed_calc(void) {
    float w_l, w_r; // 左右驱动轮转子相对大地的的角速度
    float v_l, v_r; // 左右驱动轮转子相对大地的的线速度
    float v_lb, v_rb; // 左右驱动轮计算出的机体速度

    float aver_v; // 机体速度平均值

    // 左边驱动轮转子相对大地的角速度
    w_l = ((-get_wheel_motors()->speed_rpm) / RATIO) * RPM_TO_RAD_PER_S +
          chassis.leg_L.vmc.forward_kinematics.fk_phi.d_phi0 - chassis.imu_reference.pitch_gyro;
    // 轮毂相对于机体(b系)的速度
    v_l = w_l * chassis_physical_config.wheel_radius;

    // 右边驱动轮转子相对大地角速度
    w_r = ((get_wheel_motors() + 1)->speed_rpm / RATIO) * RPM_TO_RAD_PER_S +
          chassis.leg_R.vmc.forward_kinematics.fk_phi.d_phi0 - chassis.imu_reference.pitch_gyro;
    // 轮毂相对于机体(b系)的速度
    v_r = w_r * chassis_physical_config.wheel_radius;

    v_lb = v_l + chassis.leg_L.vmc.forward_kinematics.fk_L0.L0_dot * sinf(chassis.leg_L.state_variable_feedback.theta) +
           chassis.leg_L.vmc.forward_kinematics.fk_L0.L0 * chassis.leg_L.state_variable_feedback.theta_dot *
           cosf(chassis.leg_L.state_variable_feedback.theta);

    v_rb = v_r + chassis.leg_R.vmc.forward_kinematics.fk_L0.L0_dot * sinf(chassis.leg_R.state_variable_feedback.theta) +
           chassis.leg_R.vmc.forward_kinematics.fk_L0.L0 * chassis.leg_R.state_variable_feedback.theta_dot *
           cosf(chassis.leg_R.state_variable_feedback.theta);

    aver_v = (v_lb + v_rb) / 2;

    Speed_EstimateKF_Update(&Speed_EstimateKF, chassis.imu_reference.robot_ax, aver_v);//不断更新卡尔曼滤波中的各项参数
}
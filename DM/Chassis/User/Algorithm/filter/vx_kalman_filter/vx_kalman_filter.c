#include "vx_kalman_filter.h"
#include "wheel.h"
#include "ins_task.h"
#include "robot_def.h"
#include "vofa.h"

/** 速度融合 **/
KalmanFilter_t vaEstimateKF;	   // 卡尔曼滤波器结构体

float vel_acc[2]; // 轮毂速度与加速度融合后的结果

float vaEstimateKF_F[4] = {1.0f, 0.002f,
                           0.0f, 1.0f};	   // 状态转移矩阵，控制周期为0.002s

float vaEstimateKF_P[4] = {1.0f, 0.0f,
                           0.0f, 1.0f};    // 后验估计协方差初始值

float vaEstimateKF_Q[4] = {VEL_PROCESS_NOISE, 0.0f,
                           0.0f, ACC_PROCESS_NOISE};    // Q矩阵初始值、先验估计值方差噪声

float vaEstimateKF_R[4] = {VEL_MEASURE_NOISE, 0.0f,
                           0.0f,  ACC_MEASURE_NOISE}; 	//200、200为测量噪声方差

const float vaEstimateKF_H[4] = {1.0f, 0.0f,
                                 0.0f, 1.0f};	// 设置矩阵H为常量


/*******************************************************************************
 *                                  速度融合                                    *
 *******************************************************************************/
void xvEstimateKF_Init(KalmanFilter_t *EstimateKF)//初始化卡尔曼结构体，并把该开头定义的矩阵复制到结构体中的矩阵
{
    Kalman_Filter_Init(EstimateKF, 2, 0, 2);	// 状态向量2维 没有控制量 测量向量2维

    memcpy(EstimateKF->F_data, vaEstimateKF_F, sizeof(vaEstimateKF_F));
    memcpy(EstimateKF->P_data, vaEstimateKF_P, sizeof(vaEstimateKF_P));
    memcpy(EstimateKF->Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
    memcpy(EstimateKF->R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
    memcpy(EstimateKF->H_data, vaEstimateKF_H, sizeof(vaEstimateKF_H));

}

static void xvEstimateKF_Update(KalmanFilter_t *EstimateKF ,float acc,float vel)
{
    //卡尔曼滤波器测量值更新
    EstimateKF->MeasuredVector[0] =	vel;//测量速度
    EstimateKF->MeasuredVector[1] = acc;//测量加速度

    //卡尔曼滤波器更新函数
    Kalman_Filter_Update(EstimateKF);

    // 提取估计值
    for (uint8_t i = 0; i < 2; i++)
    {
        vel_acc[i] = EstimateKF->FilteredValue[i];
    }
}

void speed_calc(void)
{
    float w_l, w_r = 0.0f; // 左右驱动轮转子相对大地的的角速度
    float v_l, v_r = 0.0f; // 左右驱动轮转子相对大地的的线速度
    float v_lb, v_rb = 0.0f; // 左右驱动轮计算出的机体速度

    float aver_v = 0.0f; // 机体速度平均值

    // 左边驱动轮转子相对大地的角速度
    w_l = ((-get_wheel_motors()->speed_rpm) / RATIO) * RPM_TO_RAD_PER_S + chassis.leg_L.vmc.forward_kinematics.fk_phi.d_phi0 - chassis.imu_reference.pitch_gyro;
    // 轮毂相对于机体(b系)的速度
    v_l = w_l * chassis_physical_config.wheel_radius;

    // 右边驱动轮转子相对大地角速度
    w_r = ((get_wheel_motors() + 1)->speed_rpm / RATIO) * RPM_TO_RAD_PER_S + chassis.leg_R.vmc.forward_kinematics.fk_phi.d_phi0 - chassis.imu_reference.pitch_gyro;
    // 轮毂相对于机体(b系)的速度
    v_r = w_r * chassis_physical_config.wheel_radius;

    v_lb = v_l + chassis.leg_L.vmc.forward_kinematics.fk_L0.L0_dot * sinf(chassis.leg_L.state_variable_feedback.theta) + chassis.leg_L.vmc.forward_kinematics.fk_L0.L0 * chassis.leg_L.state_variable_feedback.theta_dot * cosf(chassis.leg_L.state_variable_feedback.theta);
    v_rb = v_r + chassis.leg_R.vmc.forward_kinematics.fk_L0.L0_dot * sinf(chassis.leg_R.state_variable_feedback.theta) + chassis.leg_R.vmc.forward_kinematics.fk_L0.L0 * chassis.leg_R.state_variable_feedback.theta_dot * cosf(chassis.leg_R.state_variable_feedback.theta);

    aver_v = (v_lb + v_rb) / 2;

    vel_acc[0] = aver_v;

//    // 不融合
//    xvEstimateKF_Update(&vaEstimateKF,INS.MotionAccel_n[X],aver_v);//不断更新卡尔曼滤波中的各项参数
}
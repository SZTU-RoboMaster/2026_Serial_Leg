/**
 ******************************************************************************
 * @file    QuaternionEKF.c
 * @author  Wang Hongxi
 * @version V1.2.0
 * @date    2022/3/8
 * @brief   attitude update with gyro bias estimate and chi-square test
 ******************************************************************************
 * @attention
 * 1st order LPF transfer function:
 *     1
 *  ———————
 *  as + 1
 ******************************************************************************
 */
#include "QuaternionEKF.h"
#include <string.h>
#include <math.h>

QEKF_INS_t QEKF_INS;

const float IMU_QuaternionEKF_F[36] = {1, 0, 0, 0, 0, 0,
                                       0, 1, 0, 0, 0, 0,
                                       0, 0, 1, 0, 0, 0,
                                       0, 0, 0, 1, 0, 0,
                                       0, 0, 0, 0, 1, 0,
                                       0, 0, 0, 0, 0, 1};

float IMU_QuaternionEKF_P[36] = {100000, 0.1, 0.1, 0.1, 0.1, 0.1,
                                 0.1, 100000, 0.1, 0.1, 0.1, 0.1,
                                 0.1, 0.1, 100000, 0.1, 0.1, 0.1,
                                 0.1, 0.1, 0.1, 100000, 0.1, 0.1,
                                 0.1, 0.1, 0.1, 0.1, 100, 0.1,
                                 0.1, 0.1, 0.1, 0.1, 0.1, 100};

float IMU_QuaternionEKF_K[18];
float IMU_QuaternionEKF_H[18];

static float invSqrt(float x);
static void IMU_QuaternionEKF_Observe(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_F_Linearization_P_Fading(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_SetH(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_xhatUpdate(KalmanFilter_t *kf);

/**
 * @brief Quaternion EKF initialization and some reference value
 * @param[in] process_noise1 quaternion process noise    10
 * @param[in] process_noise2 gyro bias process noise     0.001
 * @param[in] measure_noise  accel measure noise         1000000
 * @param[in] lambda         fading coefficient          0.9996
 * @param[in] lpf            lowpass filter coefficient  0
 */
void IMU_QuaternionEKF_Init(float process_noise1, float process_noise2, float measure_noise, float lambda, float lpf)
{
    QEKF_INS.Initialized = 1;
    QEKF_INS.Q1 = process_noise1;
    QEKF_INS.Q2 = process_noise2;
    QEKF_INS.R = measure_noise;
    QEKF_INS.ChiSquareTestThreshold = 1e-8f;
    QEKF_INS.ConvergeFlag = 0;
    QEKF_INS.ErrorCount = 0;
    QEKF_INS.UpdateCount = 0;
    QEKF_INS.StableFlag = 0;
    QEKF_INS.AdaptiveGainScale = 1.0f;
    QEKF_INS.YawRoundCount = 0;
    QEKF_INS.YawAngleLast = 0;

    if (lambda > 1) {
        lambda = 1;
    }
    QEKF_INS.lambda = lambda;
    QEKF_INS.accLPFcoef = lpf;

    // 初始化卡尔曼滤波器
    Kalman_Filter_Init(&QEKF_INS.IMU_QuaternionEKF, 6, 0, 3);
    Matrix_Init(&QEKF_INS.ChiSquare, 1, 1, QEKF_INS.ChiSquare_Data);

    // 姿态初始化（单位四元数）
    QEKF_INS.IMU_QuaternionEKF.xhat_data[0] = 1;  // q0
    QEKF_INS.IMU_QuaternionEKF.xhat_data[1] = 0;  // q1
    QEKF_INS.IMU_QuaternionEKF.xhat_data[2] = 0;  // q2
    QEKF_INS.IMU_QuaternionEKF.xhat_data[3] = 0;  // q3
    QEKF_INS.IMU_QuaternionEKF.xhat_data[4] = 0;  // gyro bias x
    QEKF_INS.IMU_QuaternionEKF.xhat_data[5] = 0;  // gyro bias y

    // 设置自定义函数
    QEKF_INS.IMU_QuaternionEKF.User_Func0_f = IMU_QuaternionEKF_Observe;
    QEKF_INS.IMU_QuaternionEKF.User_Func1_f = IMU_QuaternionEKF_F_Linearization_P_Fading;
    QEKF_INS.IMU_QuaternionEKF.User_Func2_f = IMU_QuaternionEKF_SetH;
    QEKF_INS.IMU_QuaternionEKF.User_Func3_f = IMU_QuaternionEKF_xhatUpdate;

    // 跳过标准步骤中的部分计算
    QEKF_INS.IMU_QuaternionEKF.SkipEq3 = 1;
    QEKF_INS.IMU_QuaternionEKF.SkipEq4 = 1;

    // 复制初始矩阵
    memcpy(QEKF_INS.IMU_QuaternionEKF.F_data, IMU_QuaternionEKF_F, sizeof(IMU_QuaternionEKF_F));
    memcpy(QEKF_INS.IMU_QuaternionEKF.P_data, IMU_QuaternionEKF_P, sizeof(IMU_QuaternionEKF_P));

    // 初始化运动加速度
    QEKF_INS.MotionAccel_b[0] = 0;
    QEKF_INS.MotionAccel_b[1] = 0;
    QEKF_INS.MotionAccel_b[2] = 0;
    QEKF_INS.MotionAccel_n[0] = 0;
    QEKF_INS.MotionAccel_n[1] = 0;
    QEKF_INS.MotionAccel_n[2] = 0;

    // 初始化导航系基向量
    QEKF_INS.xn[0] = 1; QEKF_INS.xn[1] = 0; QEKF_INS.xn[2] = 0;
    QEKF_INS.yn[0] = 0; QEKF_INS.yn[1] = 1; QEKF_INS.yn[2] = 0;
    QEKF_INS.zn[0] = 0; QEKF_INS.zn[1] = 0; QEKF_INS.zn[2] = 1;

    // 初始化陀螺仪零偏
    QEKF_INS.GyroBias[0] = 0;
    QEKF_INS.GyroBias[1] = 0;
    QEKF_INS.GyroBias[2] = 0;

    // 初始化四元数
    QEKF_INS.q[0] = 1;
    QEKF_INS.q[1] = 0;
    QEKF_INS.q[2] = 0;
    QEKF_INS.q[3] = 0;
}

/**
 * @brief Quaternion EKF update
 * @param[in]       gyro x y z in rad/s
 * @param[in]       accel x y z in m/s?
 * @param[in]       update period in s
 */
void IMU_QuaternionEKF_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    static float halfgxdt, halfgydt, halfgzdt;
    static float accelInvNorm;
    KalmanFilter_t *kf = &QEKF_INS.IMU_QuaternionEKF;

    if (!QEKF_INS.Initialized) {
        IMU_QuaternionEKF_Init(10, 0.001f, 10000000.0f, 0.9996f, 0.0085f);
        return;
    }

    /*   F, number with * represent vals to be set
     0      1*     2*     3*     4     5
     6*     7      8*     9*    10    11
    12*    13*    14     15*    16    17
    18*    19*    20*    21     22    23
    24     25     26     27     28    29
    30     31     32     33     34    35
    */
    QEKF_INS.dt = dt;

    // 去除零偏后的陀螺仪数据
    QEKF_INS.Gyro[0] = gx - QEKF_INS.GyroBias[0];
    QEKF_INS.Gyro[1] = gy - QEKF_INS.GyroBias[1];
    QEKF_INS.Gyro[2] = gz - QEKF_INS.GyroBias[2];

    // 计算用于状态转移矩阵的参数
    halfgxdt = 0.5f * QEKF_INS.Gyro[0] * dt;
    halfgydt = 0.5f * QEKF_INS.Gyro[1] * dt;
    halfgzdt = 0.5f * QEKF_INS.Gyro[2] * dt;

    // 重置状态转移矩阵
    memcpy(kf->F_data, IMU_QuaternionEKF_F, sizeof(IMU_QuaternionEKF_F));

    // 设置状态转移矩阵的左上角4x4子矩阵
    kf->F_data[1] = -halfgxdt;
    kf->F_data[2] = -halfgydt;
    kf->F_data[3] = -halfgzdt;

    kf->F_data[6] = halfgxdt;
    kf->F_data[8] = halfgzdt;
    kf->F_data[9] = -halfgydt;

    kf->F_data[12] = halfgydt;
    kf->F_data[13] = -halfgzdt;
    kf->F_data[15] = halfgxdt;

    kf->F_data[18] = halfgzdt;
    kf->F_data[19] = halfgydt;
    kf->F_data[20] = -halfgxdt;

    // 加速度低通滤波
    if (QEKF_INS.UpdateCount == 0) {
        // 第一次更新，直接赋值
        QEKF_INS.Accel[0] = ax;
        QEKF_INS.Accel[1] = ay;
        QEKF_INS.Accel[2] = az;
    } else {
        // 一阶低通滤波
        float coef = QEKF_INS.accLPFcoef / (dt + QEKF_INS.accLPFcoef);
        QEKF_INS.Accel[0] = QEKF_INS.Accel[0] * coef + ax * (1.0f - coef);
        QEKF_INS.Accel[1] = QEKF_INS.Accel[1] * coef + ay * (1.0f - coef);
        QEKF_INS.Accel[2] = QEKF_INS.Accel[2] * coef + az * (1.0f - coef);
    }

    // 归一化加速度作为观测值
    accelInvNorm = invSqrt(QEKF_INS.Accel[0] * QEKF_INS.Accel[0] +
                           QEKF_INS.Accel[1] * QEKF_INS.Accel[1] +
                           QEKF_INS.Accel[2] * QEKF_INS.Accel[2]);

    for (uint8_t i = 0; i < 3; i++) {
        kf->MeasuredVector[i] = QEKF_INS.Accel[i] * accelInvNorm;
    }

    // 计算角速度和加速度模长
    QEKF_INS.gyro_norm = sqrtf(QEKF_INS.Gyro[0] * QEKF_INS.Gyro[0] +
                                QEKF_INS.Gyro[1] * QEKF_INS.Gyro[1] +
                                QEKF_INS.Gyro[2] * QEKF_INS.Gyro[2]);

    QEKF_INS.accl_norm = 1.0f / accelInvNorm;

    // 判断运动稳定性（用于卡方检验）
    if (QEKF_INS.gyro_norm < 0.3f &&
        QEKF_INS.accl_norm > 9.3f &&
        QEKF_INS.accl_norm < 10.3f) {
        QEKF_INS.StableFlag = 1;
    } else {
        QEKF_INS.StableFlag = 0;
    }

    // 设置过程噪声矩阵Q
    kf->Q_data[0] = QEKF_INS.Q1 * dt;
    kf->Q_data[7] = QEKF_INS.Q1 * dt;
    kf->Q_data[14] = QEKF_INS.Q1 * dt;
    kf->Q_data[21] = QEKF_INS.Q1 * dt;
    kf->Q_data[28] = QEKF_INS.Q2 * dt;
    kf->Q_data[35] = QEKF_INS.Q2 * dt;

    // 设置观测噪声矩阵R
    kf->R_data[0] = QEKF_INS.R;
    kf->R_data[4] = QEKF_INS.R;
    kf->R_data[8] = QEKF_INS.R;

    // 执行卡尔曼滤波更新
    Kalman_Filter_Update(kf);

    // 获取滤波后的结果
    QEKF_INS.q[0] = kf->FilteredValue[0];
    QEKF_INS.q[1] = kf->FilteredValue[1];
    QEKF_INS.q[2] = kf->FilteredValue[2];
    QEKF_INS.q[3] = kf->FilteredValue[3];
    QEKF_INS.GyroBias[0] = kf->FilteredValue[4];
    QEKF_INS.GyroBias[1] = kf->FilteredValue[5];
    QEKF_INS.GyroBias[2] = 0;  // Z轴零偏不可观测

    // 四元数转欧拉角
    QEKF_INS.Roll = asinf(-2.0f * (QEKF_INS.q[1] * QEKF_INS.q[3] - QEKF_INS.q[0] * QEKF_INS.q[2])) * 57.295779513f;
    QEKF_INS.Pitch = atan2f(2.0f * (QEKF_INS.q[0] * QEKF_INS.q[1] + QEKF_INS.q[2] * QEKF_INS.q[3]),
                            2.0f * (QEKF_INS.q[0] * QEKF_INS.q[0] + QEKF_INS.q[3] * QEKF_INS.q[3]) - 1.0f) * 57.295779513f;
    QEKF_INS.Yaw = atan2f(2.0f * (QEKF_INS.q[0] * QEKF_INS.q[3] + QEKF_INS.q[1] * QEKF_INS.q[2]),
                          2.0f * (QEKF_INS.q[0] * QEKF_INS.q[0] + QEKF_INS.q[1] * QEKF_INS.q[1]) - 1.0f) * 57.295779513f;

    // 处理连续Yaw角
    if (QEKF_INS.Yaw - QEKF_INS.YawAngleLast > 180.0f) {
        QEKF_INS.YawRoundCount--;
    } else if (QEKF_INS.Yaw - QEKF_INS.YawAngleLast < -180.0f) {
        QEKF_INS.YawRoundCount++;
    }
    QEKF_INS.YawTotalAngle = 360.0f * QEKF_INS.YawRoundCount + QEKF_INS.Yaw;
    QEKF_INS.YawAngleLast = QEKF_INS.Yaw;

    QEKF_INS.UpdateCount++;
}

/**
 * @brief 用于更新线性化后的状态转移矩阵F右上角的一个4x2分块矩阵,稍后用于协方差矩阵P的更新;
 *        并对零漂的方差进行限制,防止过度收敛并限幅防止发散
 *
 * @param kf
 */
static void IMU_QuaternionEKF_F_Linearization_P_Fading(KalmanFilter_t *kf)
{
    float q0, q1, q2, q3;
    float qInvNorm;

    q0 = kf->xhatminus_data[0];
    q1 = kf->xhatminus_data[1];
    q2 = kf->xhatminus_data[2];
    q3 = kf->xhatminus_data[3];

    // 四元数归一化
    qInvNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    kf->xhatminus_data[0] *= qInvNorm;
    kf->xhatminus_data[1] *= qInvNorm;
    kf->xhatminus_data[2] *= qInvNorm;
    kf->xhatminus_data[3] *= qInvNorm;

    /*  F, number with * represent vals to be set
     0     1     2     3     4*     5*
     6     7     8     9    10*    11*
    12    13    14    15    16*    17*
    18    19    20    21    22*    23*
    24    25    26    27    28     29
    30    31    32    33    34     35
    */
    // 设置F矩阵右上角的4x2子矩阵（零偏相关）
    kf->F_data[4] = q1 * QEKF_INS.dt / 2.0f;
    kf->F_data[5] = q2 * QEKF_INS.dt / 2.0f;

    kf->F_data[10] = -q0 * QEKF_INS.dt / 2.0f;
    kf->F_data[11] = q3 * QEKF_INS.dt / 2.0f;

    kf->F_data[16] = -q3 * QEKF_INS.dt / 2.0f;
    kf->F_data[17] = -q0 * QEKF_INS.dt / 2.0f;

    kf->F_data[22] = q2 * QEKF_INS.dt / 2.0f;
    kf->F_data[23] = -q1 * QEKF_INS.dt / 2.0f;

    // 渐消因子处理（防止零偏过度收敛）
    kf->P_data[28] /= QEKF_INS.lambda;
    kf->P_data[35] /= QEKF_INS.lambda;

    // 协方差限幅
    if (kf->P_data[28] > 10000.0f) kf->P_data[28] = 10000.0f;
    if (kf->P_data[35] > 10000.0f) kf->P_data[35] = 10000.0f;
}

/**
 * @brief 在工作点处计算观测函数h(x)的Jacobi矩阵H
 *
 * @param kf
 */
static void IMU_QuaternionEKF_SetH(KalmanFilter_t *kf)
{
    float doubleq0, doubleq1, doubleq2, doubleq3;

    doubleq0 = 2.0f * kf->xhatminus_data[0];
    doubleq1 = 2.0f * kf->xhatminus_data[1];
    doubleq2 = 2.0f * kf->xhatminus_data[2];
    doubleq3 = 2.0f * kf->xhatminus_data[3];

    /* H
     0     1     2     3     4     5
     6     7     8     9    10    11
    12    13    14    15    16    17
    last two cols are zero
    */
    // 清空H矩阵
    memset(kf->H_data, 0, sizeof(float) * kf->zSize * kf->xhatSize);

    // 设置H矩阵（加速度对四元数的雅可比）
    kf->H_data[0] = -doubleq2;
    kf->H_data[1] = doubleq3;
    kf->H_data[2] = -doubleq0;
    kf->H_data[3] = doubleq1;

    kf->H_data[6] = doubleq1;
    kf->H_data[7] = doubleq0;
    kf->H_data[8] = doubleq3;
    kf->H_data[9] = doubleq2;

    kf->H_data[12] = doubleq0;
    kf->H_data[13] = -doubleq1;
    kf->H_data[14] = -doubleq2;
    kf->H_data[15] = doubleq3;
}

/**
 * @brief 利用观测值和先验估计得到最优的后验估计
 *        加入了卡方检验以判断融合加速度的条件是否满足
 *        同时引入发散保护保证恶劣工况下的必要量测更新
 *
 * @param kf
 */
static void IMU_QuaternionEKF_xhatUpdate(KalmanFilter_t *kf)
{
    float q0, q1, q2, q3;

    // 计算卡尔曼增益相关矩阵
    Matrix_Transpose(&kf->H, &kf->HT);

    // S = H * P' * H' + R
    kf->temp_matrix.numRows = kf->H.numRows;
    kf->temp_matrix.numCols = kf->Pminus.numCols;
    Matrix_Multiply(&kf->H, &kf->Pminus, &kf->temp_matrix);

    kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
    kf->temp_matrix1.numCols = kf->HT.numCols;
    Matrix_Multiply(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1);

    Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S);
    Matrix_Inverse(&kf->S, &kf->temp_matrix1);  // inv(S)

    // 获取先验状态
    q0 = kf->xhatminus_data[0];
    q1 = kf->xhatminus_data[1];
    q2 = kf->xhatminus_data[2];
    q3 = kf->xhatminus_data[3];

    // 计算预测的重力方向
    kf->temp_vector_data[0] = 2.0f * (q1 * q3 - q0 * q2);
    kf->temp_vector_data[1] = 2.0f * (q0 * q1 + q2 * q3);
    kf->temp_vector_data[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    // 计算方向余弦
    for (uint8_t i = 0; i < 3; i++) {
        QEKF_INS.OrientationCosine[i] = acosf(fabsf(kf->temp_vector_data[i]));
    }

    // 计算残差: z - h(x)
    Matrix_Subtract(&kf->z, &kf->temp_vector, &kf->temp_vector1);

    // 卡方检验
    kf->temp_matrix.numRows = kf->temp_vector1.numRows;
    kf->temp_matrix.numCols = 1;
    Matrix_Multiply(&kf->temp_matrix1, &kf->temp_vector1, &kf->temp_matrix);

    Matrix_Transpose(&kf->temp_vector1, &kf->temp_vector);
    Matrix_Multiply(&kf->temp_vector, &kf->temp_matrix, &QEKF_INS.ChiSquare);

    // 根据卡方检验结果决定更新策略
    if (QEKF_INS.ChiSquare_Data[0] < 0.5f * QEKF_INS.ChiSquareTestThreshold) {
        QEKF_INS.ConvergeFlag = 1;
    }

    if (QEKF_INS.ChiSquare_Data[0] > QEKF_INS.ChiSquareTestThreshold && QEKF_INS.ConvergeFlag) {
        if (QEKF_INS.StableFlag) {
            QEKF_INS.ErrorCount++;
        } else {
            QEKF_INS.ErrorCount = 0;
        }

        if (QEKF_INS.ErrorCount > 50) {
            // 滤波器发散
            QEKF_INS.ConvergeFlag = 0;
            kf->SkipEq5 = 0;
        } else {
            // 残差过大，仅用预测
            memcpy(kf->xhat_data, kf->xhatminus_data, sizeof(float) * kf->xhatSize);
            memcpy(kf->P_data, kf->Pminus_data, sizeof(float) * kf->xhatSize * kf->xhatSize);
            kf->SkipEq5 = 1;
            return;
        }
    } else {
        // 自适应增益调节
        if (QEKF_INS.ChiSquare_Data[0] > 0.1f * QEKF_INS.ChiSquareTestThreshold && QEKF_INS.ConvergeFlag) {
            QEKF_INS.AdaptiveGainScale = (QEKF_INS.ChiSquareTestThreshold - QEKF_INS.ChiSquare_Data[0]) / (0.9f * QEKF_INS.ChiSquareTestThreshold);
        } else {
            QEKF_INS.AdaptiveGainScale = 1.0f;
        }
        QEKF_INS.ErrorCount = 0;
        kf->SkipEq5 = 0;
    }

    // 计算卡尔曼增益 K
    kf->temp_matrix.numRows = kf->Pminus.numRows;
    kf->temp_matrix.numCols = kf->HT.numCols;
    Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->temp_matrix);
    Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K);

    // 自适应增益调整
    for (uint8_t i = 0; i < kf->K.numRows * kf->K.numCols; i++) {
        kf->K_data[i] *= QEKF_INS.AdaptiveGainScale;
    }

    // 方向余弦约束
    for (uint8_t i = 4; i < 6; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            kf->K_data[i * 3 + j] *= QEKF_INS.OrientationCosine[i - 4] / 1.5707963f;  // pi/2
        }
    }

    // 状态更新
    kf->temp_vector.numRows = kf->K.numRows;
    kf->temp_vector.numCols = 1;
    Matrix_Multiply(&kf->K, &kf->temp_vector1, &kf->temp_vector);

    // 零漂修正限幅
    if (QEKF_INS.ConvergeFlag) {
        for (uint8_t i = 4; i < 6; i++) {
            if (kf->temp_vector.pData[i] > 1e-2f * QEKF_INS.dt) {
                kf->temp_vector.pData[i] = 1e-2f * QEKF_INS.dt;
            }
            if (kf->temp_vector.pData[i] < -1e-2f * QEKF_INS.dt) {
                kf->temp_vector.pData[i] = -1e-2f * QEKF_INS.dt;
            }
        }
    }

    // 不修正yaw轴数据
    kf->temp_vector.pData[3] = 0;

    // 后验状态 = 先验状态 + K * 残差
    Matrix_Add(&kf->xhatminus, &kf->temp_vector, &kf->xhat);
}

/**
 * @brief EKF观测环节,其实就是把数据复制一下
 *
 * @param kf kf类型定义
 */
static void IMU_QuaternionEKF_Observe(KalmanFilter_t *kf)
{
    memcpy(IMU_QuaternionEKF_P, kf->P_data, sizeof(IMU_QuaternionEKF_P));
    memcpy(IMU_QuaternionEKF_K, kf->K_data, sizeof(IMU_QuaternionEKF_K));
    memcpy(IMU_QuaternionEKF_H, kf->H_data, sizeof(IMU_QuaternionEKF_H));
}

/**
 * @brief 自定义1/sqrt(x),速度更快
 *
 * @param x x
 * @return float
 */
static float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f375a86 - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}
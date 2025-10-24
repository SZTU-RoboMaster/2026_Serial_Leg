/*遥控器键位：

        S1                                                                         S0
                    |                                                   |
                    |                                                   |
                    |                                                   |
                    |                                                   |
           ------------------ 2                                ------------------ 0
                    |                                                   |
                    |                                                   |
                    |                                                   |
                    3                                                   1

*/



#include "board_communication_task.h"
#include "remote.h"
#include "robot_def.h"

Gimbal_Unpack_Data gimbal_unpack_data;

static float uint_to_float(int x, float x_min, float x_max, int bits) {
    /// 将无符号整数转换回浮点数，还原float_to_uint的映射过程 ///
    float span = x_max - x_min;
    float max_val = (1 << bits) - 1;  // 目标整数的最大值
    return x_min + (x / max_val) * span;
}


void Gimbal_Data_Unpack(const uint8_t *rx_data) {

    /** 解析前后通道 **/
    gimbal_unpack_data.vx_channel.data[0] = rx_data[0];
    gimbal_unpack_data.vx_channel.data[1] = rx_data[1];

    /** 解析腿长通道 **/
    gimbal_unpack_data.leg_channel.data[0] = rx_data[2];
    gimbal_unpack_data.leg_channel.data[1] = rx_data[3];

    /** 解析右拨钮 **/
    gimbal_unpack_data.sr = rx_data[4];

    /** 解析yaw与底盘正方向相对角度 **/
    int16_t yaw_relative_angle_int = (int16_t)((rx_data[6] << 8) | rx_data[5]);
    uint16_t yaw_relative_angle_uint = (uint16_t)yaw_relative_angle_int;
    gimbal_unpack_data.yaw_relative_angle = uint_to_float(yaw_relative_angle_uint, -180.0f, 180.0f, 16);


}



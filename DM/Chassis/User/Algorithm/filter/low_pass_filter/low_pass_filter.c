#include "low_pass_filter.h"

void low_pass_filter_init(LowPassFilter *filter, float alpha) {
    // 初始化滤波器的系数
    filter->alpha = alpha;

    // 初始化上一时刻的值为0
    filter->prev_value = 0;
}

float update_low_pass_filter(LowPassFilter *filter, float value) {
    // 计算滤波后的值
    float filted_value = filter->alpha * value + (1 - filter->alpha) * filter->prev_value;

    // 更新上一时刻的值
    filter->prev_value = filted_value;

    return filted_value;
}

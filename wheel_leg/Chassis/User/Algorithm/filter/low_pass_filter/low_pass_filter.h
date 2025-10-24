#ifndef LOW_PASS_FILTER_H
#define LOW_PASS_FILTER_H

// 定义低通滤波器结构体
typedef struct {
    float alpha;         // 滤波系数
    float prev_value;   // 上一时刻的值
} LowPassFilter;

// 初始化低通滤波器
void low_pass_filter_init(LowPassFilter *filter, float alpha);

// 更新低通滤波器状态并返回滤波后的值
float update_low_pass_filter(LowPassFilter *filter, float value);

#endif


#include "moving_filter.h"

void moving_average_filter_init(MovingAverageFilter *filter) {
  // 将窗口数组中的元素初始化为0
  for (int i = 0; i < WINDOW_SIZE; i++) {
    filter->window[i] = 0;
  }

  // 初始化其他变量
  filter->index = 0;
  filter->sum = 0;
}

// 更新滤波器状态
float update_moving_average_filter(MovingAverageFilter *filter, float newValue) {
  // 减去旧值
  filter->sum -= filter->window[filter->index];

  // 添加新值
  filter->window[filter->index] = newValue;
  filter->sum += newValue;

  // 更新索引
  filter->index = (filter->index + 1) % WINDOW_SIZE;

  return filter->sum / WINDOW_SIZE;
}

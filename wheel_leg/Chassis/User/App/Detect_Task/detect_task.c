//#include "detect_task.h"
//
//
////检测离线的设备
//detect_device_t detect_list[DETECT_DEVICE_LIST_LEN];
//
//static void detect_init(uint16_t index, uint32_t threshold_time, uint8_t warning_level)
//{
//    detect_list[index].status = OFFLINE;//默认先离线
//    detect_list[index].last_online_time = HAL_GetTick();
//    detect_list[index].offline_threshold = threshold_time;//离线判断阈值
//    detect_list[index].warning_level = warning_level;//警告等级
//
//}
//
//void detect_handle(uint8_t index)
//{
//    detect_list[index].last_online_time = HAL_GetTick();
//}
//
//void detect_task(void const *pvParameters) {
////    vTaskDelay(DETECT_TASK_INIT_TIME);
//
//    //1级的话未必要等级,1级以上需要控制车不要发癫
//    detect_init(DETECT_CAP, 200, 1);
//
//    // 关节
//    for (uint8_t i = CHASSIS_JOINT_LF; i <= CHASSIS_JOINT_RB; i++)
//    {
//        detect_init(i, 200,` 2);//200ms
//    }
//
//    // 轮毂
//    for (uint8_t i = CHASSIS_WHEEL_L; i <= CHASSIS_WHEEL_R; i++)
//    {
//        detect_init(i, 200, 2);//200ms
//    }
//
//    uint8_t offline_num = 0;//离线设备数量
//    uint32_t max_level = 0;//0表示最低警告等级 也就是不警告
//
//    while (1) {
//        offline_num = 0;
//        max_level = 0;
//
//        for (uint8_t i = 0; i < 6; i++)
//            same_level_count[i] = 0;
//
//        for (uint8_t i = 0; i < DETECT_DEVICE_LIST_LEN; i++) {
//            if ((HAL_GetTick() - detect_list[i].last_online_time) >
//                detect_list[i].offline_threshold) {
//                detect_list[i].status = OFFLINE;
//                offline_num++;//离线设备数量
//                //记录最高提示等级
//                max_level = detect_list[i].warning_level >= max_level ?
//                            detect_list[i].warning_level : max_level;
//
//                same_level_count[detect_list[i].warning_level]++;
//            } else {
//                detect_list[i].status = ONLINE;
//            }
//        }
//
//        offline_remind(offline_num, max_level, same_level_count[max_level]);
//
//        vTaskDelay(40);
//    }
//}
cmake_minimum_required(VERSION 3.22)

# 仅生成一次，用户可按需修改
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# 构建类型
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif ()

# 项目名称
set(CMAKE_PROJECT_NAME wheel_leg)

# 引入工具链
include("cmake/gcc-arm-none-eabi.cmake")

# 生成compile_commands.json，方便clangd索引
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# 核心项目配置
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# 启用C和汇编
enable_language(C ASM)

# 创建可执行文件
add_executable(${CMAKE_PROJECT_NAME})

# 引入CubeMX生成的代码
add_subdirectory(cmake/stm32cubemx)

# -------------------------- 新增：CMSIS-DSP 源码编译配置 --------------------------
# 1. 递归查找所有CMSIS-DSP源码（适配M7，包含矩阵、三角函数等所有模块）
file(GLOB_RECURSE CMSIS_DSP_SOURCES
        ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Source/*.c
)

# 2. 编译DSP源码为静态库（避免预编译库适配问题）
add_library(cmsis_dsp STATIC ${CMSIS_DSP_SOURCES})

# 3. DSP库编译宏（必须匹配Cortex-M7 + FPU）
target_compile_definitions(cmsis_dsp PRIVATE
        ARM_MATH_CM7          # M7内核专属宏
        ARM_MATH_MATRIX_CHECK # 可选：矩阵边界检查
        ARM_MATH_ROUNDING     # 可选：启用舍入
        __FPU_PRESENT=1U      # 声明FPU存在（M7必加）
)

# 4. DSP库头文件路径（适配CubeMX生成的目录结构）
target_include_directories(cmsis_dsp PRIVATE
        ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Core/Include
        ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Include
        ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32H7xx/Include # 替换为你的芯片型号（如STM32H743xx）
)

# -------------------------- 原有配置修正 --------------------------
# 链接目录：删除无效的DSP目录（预编译库不用了）
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
        # 保留用户自定义库路径（如有）
)

# 添加用户源码（保留你的原有配置）
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        User/Algorithm/controller/controller.c
        User/Algorithm/filter/kalman_filter/kalman_filter.c
        User/Algorithm/filter/low_pass_filter/low_pass_filter.c
        User/Algorithm/filter/moving_filter/moving_filter.c
        User/Algorithm/filter/vx_kalman_filter/vx_kalman_filter.c
        User/Algorithm/lqr/lqr.c
        User/Algorithm/pid/pid.c
        User/Algorithm/QuaternionEKF/QuaternionEKF.c
        User/Algorithm/user_lib/user_lib.c
        User/Algorithm/vmc/vmc.c

        User/App/Board_Communication_Task/board_communication_task.c
        User/App/Chassis_Task/chassis_task.c
        User/App/Detect_Task/detect_task.c

        User/Bsp/can/bsp_can.c
        User/Bsp/dwt/bsp_dwt.c
        User/Bsp/usart/bsp_usart.c

        User/Device/motor/joint/DM8009P/dm_8009p.c
        User/Device/motor/joint/joint.c
        User/Device/motor/wheel/DJI/DJI_motor.c
        User/Device/motor/wheel/wheel.c
        User/Device/remote/error.c
        User/Device/remote/remote.c
        User/Device/vofa/vofa.c
        User/Device/DM_IMU/dm_imu.c

        User/Robot_def/robot_def.c
)

# 头文件路径（保留你的原有配置）
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        User/Algorithm/controller
        User/Algorithm/filter/kalman_filter
        User/Algorithm/filter/low_pass_filter
        User/Algorithm/filter/moving_filter
        User/Algorithm/filter/vx_kalman_filter
        User/Algorithm/lqr
        User/Algorithm/pid
        User/Algorithm/QuaternionEKF
        User/Algorithm/user_lib
        User/Algorithm/vmc

        User/App/Board_Communication_Task
        User/App/Chassis_Task
        User/App/Detect_Task

        User/Bsp/can
        User/Bsp/dwt
        User/Bsp/usart

        User/Device/can_device
        User/Device/DM_IMU
        User/Device/motor/joint
        User/Device/motor/joint/DM8009P
        User/Device/motor/wheel
        User/Device/motor/wheel/DJI
        User/Device/remote
        User/Device/vofa

        User/Robot_def
        # 新增：DSP头文件（代码中需包含arm_math.h）
        ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Include
)

# 编译宏（新增DSP必需的宏 + 保留用户自定义）
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        # DSP核心宏（M7必加）
        ARM_MATH_CM7
        __FPU_PRESENT=1U
        # 保留用户自定义宏（如有）
)

# 链接库：删除预编译的arm_cortexM7lfdp_math，替换为源码编译的cmsis_dsp
target_link_libraries(${CMAKE_PROJECT_NAME}
        stm32cubemx
        cmsis_dsp  # 核心：链接源码编译的DSP库
        # 保留用户自定义库（如有）
)

# -------------------------- 修正FPU编译选项（M7完整配置） --------------------------
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mthumb -mthumb-interwork -march=armv7e-m -mtune=cortex-m7 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -O2")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mthumb -mthumb-interwork -march=armv7e-m -mtune=cortex-m7 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -O2")
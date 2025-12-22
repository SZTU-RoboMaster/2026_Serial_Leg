cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif ()

# Set the project name
set(CMAKE_PROJECT_NAME wheel_leg)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user sources here
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

        User/Device/bmi088/BMI088driver.c
        User/Device/bmi088/BMI088Middleware.c
        User/Device/motor/joint/DM8009P/dm_8009p.c
        User/Device/motor/joint/joint.c
        User/Device/motor/wheel/DJI/DJI_motor.c
        User/Device/motor/wheel/wheel.c
        User/Device/remote/error.c
        User/Device/remote/remote.c
        User/Device/vofa/vofa.c

        User/Robot_def/robot_def.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined include paths
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

        User/Device/bmi088
        User/Device/can_device
        User/Device/motor/joint
        User/Device/motor/joint/DM8009P
        User/Device/motor/wheel
        User/Device/motor/wheel/DJI
        User/Device/remote
        User/Device/vofa

        User/Robot_def
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
        stm32cubemx

        # Add user defined libraries
)

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
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Tacking_car)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C CXX ASM)

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
    User/Device/motor.c
    User/Device/encoder.c
    User/Device/pid_controller.c
    User/Device/cascade_pid.c
    User/Device/motor_control.c
    User/Device/chassis.c
    User/Device/GrayscaleSensor.c
    User/Device/line_track.c
    User/Device/line_route.c
    User/Device/line_ab_curve_test.c
    User/Device/imu.c
    User/App/app_init.c
    User/App/debug_task.c
    User/App/grayscale_task.c
    User/App/imu_task.c
    User/App/vofa_task.c
    User/App/vofa_firewater.c
    User/App/ui_task.c
    User/App/vision_task.c
    User/function.c
    User/Device/oled_ssd1306.c
    User/Device/key.c
    User/Device/vision_protocol.c
    Module/motor/qd4310/Qd4310Motor.cpp
    Module/motor/qd4310/qd4310.c
    Module/motor/zdt_y42/Y42Protocol.cpp
    Module/motor/zdt_y42/Y42Motor.cpp
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_SOURCE_DIR}/User
    ${CMAKE_SOURCE_DIR}/User/App
    ${CMAKE_SOURCE_DIR}/User/Device
    ${CMAKE_SOURCE_DIR}/Module/motor/qd4310
    ${CMAKE_SOURCE_DIR}/Module/motor/zdt_y42
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

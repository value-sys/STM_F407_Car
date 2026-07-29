# MSPM0G3507 到 STM32F407VET6 移植说明

## 已迁移

- 通用 PID 控制器及增量式电机速度环。
- 差速底盘运动学、单轮支点旋转接口。
- 电机闭环、底盘、IMU、调试和 VOFA 任务框架。
- 集中模块初始化及原工程主要注释。

## STM32 适配

- PWM 使用当前工程 TIM1 CH1/CH2，ARR 为 999，双极性停止值为 500。
- 编码器继续使用当前工程 TIM4/TIM3 的硬件编码器模式。
- 编码器溢出周期从定时器 ARR 获取，测速分别使用各电机的线数、减速比和倍频参数。
- VOFA 继续使用当前工程 USART1，不移植 MSPM0 UART 驱动。
- IMU 继续使用当前工程 USART3 和 HAL 单字节中断接收；应用层补充有效帧计数和角加速度计算。
- FreeRTOS 继续使用 CubeMX CMSIS-RTOS v2 模板，不移植 TI FreeRTOS 端口。

## 暂未接入

- 灰度传感器读取、灰度循迹任务和相关控制逻辑。
- 灰度旧源码仍保留在 `User/Device`，但没有加入当前固件构建目标。

## 调试入口

上电默认 `g_emDebugMode = emDebugModeNone`，电机保持停止。可在调试器中切换：

- `emDebugModeMotorOpenLoop`：直接设置 `g_usDebugMotor1Pwm/g_usDebugMotor2Pwm`。
- `emDebugModeMotorSpeedLoop`：设置两路目标 RPM，执行编码器速度闭环。
- `emDebugModeEncoder`：保持停止，只通过 VOFA 观察测速。
- `emDebugModeImu`：保持停止，通过 VOFA 观察航向角、角速度和角加速度。
- `emDebugModeChassis`：上电停车3秒后，以 `g_fDebugChassisVy` 持续前进。
- `emDebugModeImuRotate`：设置 `g_fDebugImuRotateAngleDeg` 后执行IMU双环定角旋转。

电机方向由 `MOTORx_ENCODER_REVERSE`、`MOTORx_PWM_REVERSE` 和
`CHASSIS_RIGHT_MOTOR_REVERSE` 集中配置，上车前应先用开环低占空比逐路确认方向。

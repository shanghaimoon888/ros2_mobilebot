# R1 三轮小车 —— 基于 ROS 2 Humble 的差速驱动机器人

基于树莓派 4B (上位机) + STM32F407 (下位机) 的 R1 三轮小车。支持硬件级闭环 PID、多传感器融合、SLAM 建图与多端遥控。

## 📋 项目简介

本项目实现了一个完整的差速驱动移动机器人控制系统。采用**双核架构**，底层 STM32 负责高精度电机控制与传感器数据采集，上层树莓派运行 ROS 2 Humble，基于 `ros2_control` 框架进行状态发布与控制解算。目前已成功跑通 2D SLAM 建图。

### 主要功能

- ✅ **双核架构**：STM32F407 负责底盘闭环控制，树莓派 4B 负责 ROS 2 高层逻辑。
- ✅ **串口通信协议**：自定义帧头校验协议（`0xAA 0x55 Len Cmd Data Checksum`），稳定传输指令与传感器数据。
- ✅ **STM32 增量式 PID**：底层自带前馈与限幅，速度闭环极其稳定。
- ✅ **x4 正交解码**：STM32 硬件定时器编码器接口，抗抖动、高精度。
- ✅ **ros2_control 硬件接口**：C++ 编写的串口桥接节点，后台独立线程，不阻塞主循环。
- ✅ **IMU 姿态接入**：MPU6050 (DMP) 数据通过自定义 `0x82` 帧上传，ROS 2 端导出 `imu_sensor_broadcaster` 接口。
- ✅ **激光雷达接入**：成功接入二手 YDLIDAR X2，发布 `/scan` 话题，QV 配置调优完毕。
- ✅ **SLAM 建图**：跑通 `slam_toolbox`，TF 树 `map -> odom -> base_link -> laser_frame` 完美闭环。
- ✅ **多端遥控**：支持键盘遥控、手机网页 FPV 遥控（基于 `rosbridge` 和 `roslibjs`）。
- ✅ **离线语音控制**：Vosk 离线流式中文识别，结合语法限制直接发 `/cmd_vel`。
- ✅ **安全避障**：接入 HC-SR04 超声波，15cm 强制刹车逻辑。

## 🔧 硬件清单

| 组件 | 型号 | 说明 |
| :--- | :--- | :--- |
| 上位机 | 树莓派 4B | 运行 Ubuntu 22.04 + ROS 2 Humble |
| 下位机 | STM32F407VGT6 | 核心控制，跑 PID 与传感器外设 |
| 底盘 | R1 三轮小车 | 两个驱动轮 + 一个万向轮 |
| 电机驱动 | TB6612 双路驱动模块 | 带稳压版，支持 4.5V~15V |
| 电机 | 带编码器直流减速电机 | 11 线编码器 × 30 减速比 |
| 电池 | 12V 锂电池 | 为电机供电 |
| 激光雷达 | YDLIDAR X2 | 单线 2D 雷达 (USB CH340) |
| IMU | MPU6050 | I2C 接口，跑通 DMP 输出 Roll/Pitch/Yaw |
| 超声波 | HC-SR04 | 触发 TIM2 输入捕获测距 |
| 麦克风 | USB 麦克风 | 用于 Vosk 语音控制 |
| 摄像头 | CSI 摄像头 | 网页 FPV 视频推流 |

### 硬件接线参考

*(注：由于加入了下位机 STM32，具体引脚接线请参考 STM32 工程内的 `main.c` 或 CubeMX 配置)*
- **STM32 与树莓派**：USART2 串口通信（当前波特率 9600）。
- **STM32 与外设**：TIM3/TIM4 编码器；TIM10/TIM11 PWM；I2C2 接 MPU6050；TIM2 接超声波。

## 📁 目录结构

```text
ros2_ws/src/
├── r1_description/          # 机器人 URDF 描述 (包含雷达与 IMU 的安装位置)
├── r1_hardware_interface/   # 硬件接口插件（C++串口桥接与ROS接口导出）
├── r1_bringup/              # 启动、控制器配置与 systemd 自启脚本
├── voice_control/           # 语音控制节点（Vosk）
└── ydlidar_ros2_driver/     # 雷达驱动 (Humble 分支)
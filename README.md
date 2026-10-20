# R1 三轮小车 —— 基于 ROS 2 Humble 的差速驱动机器人

基于树莓派 4B 和 ROS 2 Humble 的 R1 三轮小车，使用 TB6612 双路驱动模块，支持编码器闭环 PID 控制、键盘遥控和语音控制。

## 📋 项目简介

本项目实现了一个完整的差速驱动移动机器人控制系统，硬件采用 R1 三轮底盘 + TB6612 驱动 + 12V 电池，软件基于 `ros2_control` 框架，包含自定义硬件接口插件、差速控制器、编码器闭环 PID 和语音控制节点。

### 主要功能

- ✅ **ros2_control 硬件接口**：自定义 C++ 插件，通过 pigpio 控制 TB6612 并读取编码器
- ✅ **差速驱动控制**：`diff_drive_controller` 将 `/cmd_vel` 解算为左右轮目标速度
- ✅ **编码器闭环 PID**：硬件接口内部实现 PID，速度跟踪更精准
- ✅ **x4 正交解码**：A/B 双相回调 + 查表法，分辨率提升 4 倍，抗抖动
- ✅ **键盘遥控**：`teleop_twist_keyboard` 实时控制
- ✅ **语音控制**：Vosk 离线中文语音识别，发布速度指令
- ✅ **加速度限制**：防止启动时翘头
- ✅ **速度耦合**：减少直线跑偏
- ✅ **后退缩放**：提高后退稳定性

## 🔧 硬件清单

| 组件 | 型号 | 说明 |
| :--- | :--- | :--- |
| 主控 | 树莓派 4B | 运行 Ubuntu 22.04 + ROS 2 Humble |
| 底盘 | R1 三轮小车 | 两个驱动轮 + 一个万向轮 |
| 电机驱动 | TB6612 双路驱动模块 | 带稳压版，支持 4.5V~15V |
| 电池 | 12V 锂电池（3C 认证） | 为电机供电 |
| 电机 | 带编码器直流减速电机 | 11 线编码器 × 30 减速比 |
| 麦克风 | USB 麦克风 | 用于语音控制 |

### 接线说明

#### TB6612 与树莓派

| TB6612 引脚 | 树莓派 GPIO (BCM) | 说明 |
| :--- | :--- | :--- |
| PWMA | 16 | 左轮 PWM |
| AIN1 | 20 | 左轮方向 A |
| AIN2 | 21 | 左轮方向 B |
| PWMB | 13 | 右轮 PWM |
| BIN1 | 19 | 右轮方向 A |
| BIN2 | 26 | 右轮方向 B |
| STBY | 3.3V | 使能（接高电平） |
| VCC | 3.3V | 逻辑电源 |
| VM | 12V 电池 | 电机电源 |
| GND | 共地 | 与树莓派 GND 相连 |

#### 编码器与树莓派

| 编码器信号 | 树莓派 GPIO (BCM) | 物理引脚 |
| :--- | :--- | :--- |
| 左轮 A 相 | 17 | Pin 11 |
| 左轮 B 相 | 27 | Pin 13 |
| 右轮 A 相 | 22 | Pin 15 |
| 右轮 B 相 | 23 | Pin 16 |
| VCC | 3.3V | 驱动板 3.3V 输出 |
| GND | GND | 驱动板 GND 输出 |

> **注意**：右轮为镜像安装，编码器方向已通过软件修正。

## 📁 目录结构
ros2_ws/src/
├── r1_description/ # 机器人 URDF 描述
│ ├── urdf/
│ │ ├── r1.urdf.xacro # 主 URDF
│ │ ├── r1_core.xacro # 车体、轮子几何/惯性
│ │ └── r1_ros2_control.xacro # ros2_control 硬件接口配置
│ ├── package.xml
│ └── CMakeLists.txt
│
├── r1_hardware_interface/ # 硬件接口插件（C++）
│ ├── include/r1_hardware_interface/
│ │ └── r1_system_hardware.hpp
│ ├── src/
│ │ └── r1_system_hardware.cpp
│ ├── r1_hardware_interface.xml
│ ├── package.xml
│ └── CMakeLists.txt
│
├── r1_bringup/ # 启动与控制器配置
│ ├── config/
│ │ └── r1_controllers.yaml
│ ├── launch/
│ │ └── r1_control.launch.py
│ ├── package.xml
│ └── CMakeLists.txt
│
└── voice_control/ # 语音控制节点（Python）
├── voice_control/
│ └── voice_control_node.py
├── package.xml
└── setup.py

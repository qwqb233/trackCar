<!--
 * @Author: qwqb233 qwqb.zhang@gmail.com
 * @Date: 2025-09-29 20:10:26
 * @LastEditors: qwqb233 qwqb.zhang@gmail.com
 * @FilePath: \course design5\README.md
 * @Description: 
-->
# 智能小车课程设计

## 项目简介
本项目为智能小车控制系统，基于STM32平台，集成电机驱动、OLED显示、超声波测距、颜色识别、寻迹、按键控制等功能模块，实现自动循迹、避障、颜色识别触发等任务。

## 主要模块
- **电机驱动与PID控制**：`motorLib`，支持四轮独立PID速度控制，以实现通过运动学模型控制麦克纳姆轮底盘全向移动。
- **OLED显示**：`iic`，显示欢迎界面、运行状态等信息。
- **超声波测距**：`US_ranging`，用于障碍物检测与距离测量。
- **颜色识别**：`colorRe`，通过TCS34725模块识别颜色，控制状态切换。
- **寻迹模块**：`traLib`，支持多路循迹传感器输入，实现路径跟踪。
- **按键与LED/RGB灯控制**：`redLib`，用于用户交互与状态指示。
- **内存池管理**：`memoryLib`，提供高效内存分配接口。

## 硬件依赖
- STM32F4系列主控板
- TCS34725颜色识别模块
- 超声波测距模块
- OLED显示屏
- 四路电机及驱动
- RGB灯、蜂鸣器、按键等外设

## 编译与运行
1. 使用Keil、STM32CubeIDE等工具打开工程，配置好相关硬件连接。
2. 编译并下载程序到STM32开发板。
3. 上电后，OLED显示欢迎界面，按下按键后进入自动循迹、避障、颜色识别等流程。
4. OLED实时显示速度、距离、时间等运行信息，RGB灯和蜂鸣器用于状态提示。

## 主要文件结构
- `Core/Src/main.c`：主流程与状态机
- `Core/Src/motorLib/`：电机控制相关
- `Core/Src/iic/`：OLED显示相关
- `Core/Src/US_ranging/`：超声波测距
- `Core/Src/traLib/`：寻迹模块
- `Core/Src/redLib/`：LED与RGB灯控制
- `Core/Src/memoryLib/`：内存池管理

## 备注
- 详细硬件连接方式请参考原理图及注释。
- 代码已模块化，便于扩展和维护。

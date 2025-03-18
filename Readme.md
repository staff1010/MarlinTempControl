# STM32 Temperature Control System
# STM32 温度控制系统

A temperature detection and PID heating control system extracted from Marlin firmware, implemented on STM32 platform. This project is currently under testing, and many features are still in development.
从 Marlin 固件中提取的温度检测和 PID 加热控制系统，在 STM32 平台上实现。该项目目前正在测试中，许多功能仍在开发中。

## Features 功能特点

- Fast GPIO operations for precise timing control
- 快速 GPIO 操作，实现精确的时序控制

- Temperature measurement using thermistor
- 使用热敏电阻进行温度测量

- PID control for stable temperature regulation
- 使用 PID 控制实现稳定的温度调节

- Support for different heating elements
- 支持不同类型的加热元件

## Hardware Requirements 硬件要求

- STM32 MCU (tested on STM32F1xx series)
- STM32 微控制器（在 STM32F1xx 系列上测试通过）

- Thermistor for temperature sensing (default: 100K NTC)
- 用于温度感测的热敏电阻（默认：100K NTC）

- MOSFET for heater control
- 用于加热控制的 MOSFET

## Pin Configuration 引脚配置

- Temperature Sensor: PB1 (ADC input)
- 温度传感器：PB1（ADC 输入）

- Heater Control: PA2 (PWM output) or PC13 (LED output)
- 加热控制：PA2（PWM 输出） 或者 PC13 (LED 输出)

## Dependencies 依赖项

- Arduino STM32 core
- Arduino STM32 核心库

- PlatformIO build system
- PlatformIO 构建系统

## Installation 安装方法

1. Clone this repository
1. 克隆此仓库

   ```bash
   git clone https://github.com/staff1010/stm32-temp-control.git
   ```

2. Open the project in PlatformIO
2. 在 PlatformIO 中打开项目

3. Build and upload to your STM32 board
3. 构建并上传到您的 STM32 开发板

## Usage 使用方法

1. Initialize the temperature control system:
1. 初始化温度控制系统：

   ```cpp
   temperature_init();
   ```

2. Set target temperature and PID parameters:
2. 设置目标温度和 PID 参数：

   ```cpp
   hotend_info_t hotend;
   hotend.target = 200;      // Target temperature 目标温度
   hotend.pid.Kp = 12.5;    // Proportional gain 比例系数
   hotend.pid.Ki = 0.8;     // Integral gain 积分系数
   hotend.pid.Kd = 45.0;    // Derivative gain 微分系数
   ```

## PID Tuning PID 调参

For optimal temperature control, you may need to tune the PID parameters:
为了获得最佳的温度控制效果，您可能需要调整 PID 参数：

1. Start with conservative values:
1. 从保守的值开始：
   - Kp = 10.0
   - Ki = 0.5
   - Kd = 40.0

2. Adjust parameters based on system response
2. 根据系统响应调整参数

## Current Status 当前状态

This project is still in testing phase. Some features may not be fully implemented or may not work as expected. Please report any issues you encounter.
该项目仍处于测试阶段。一些功能可能尚未完全实现或可能无法按预期工作。请报告您遇到的任何问题。

## Contributing 贡献

Contributions are welcome! Please feel free to submit a Pull Request or open an issue for discussion.
欢迎贡献！请随时提交 Pull Request 或开启一个 issue 进行讨论。

## License 许可证

This project is licensed under the GPL-3.0 License - see the LICENSE file for details.
本项目采用 GPL-3.0 许可证 - 详见 LICENSE 文件。

## Acknowledgments 致谢

- Based on Marlin Firmware temperature control system
- 基于 Marlin 固件的温度控制系统

- Thanks to the Marlin development team
- 感谢 Marlin 开发团队

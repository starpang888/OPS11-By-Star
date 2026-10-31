# STM32 IMU 与磁编码器

STM32F407VET6、FreeRTOS、Epson M-G354PDH0，使用 x-io Fusion 输出四元数和 Roll/Pitch/Yaw。
本版完成软件接入及离线回归，板上采样和启动标定正在调试，动态角度精度尚未验证。

已接入两个 MT6826S 编码器，分别使用 SPI1/PA4 和 SPI2/PB12，各自用独立任务进行标称 1000 Hz 单圈角度采样，提供 CRC 校验、报警统计和实际读取频率。
两路已改为 SPI 收发 DMA、软件片选、完成中断通知任务，支持超时及错误恢复；新增 `g_encoder/g_encoder2.dma` 调试统计。DMA 版本编译通过，待上板确认。
用户已反馈双路角度数据正常；当前新增各自的多圈累计位移及带符号转速，运动计算部分待烧录确认。详见 [MT6826S 接入说明](modules/mt6826s/README.md)。

已加入第一版双随动轮平面里程计：SPI1 测前后、SPI2 测左右，两轮直径 58 mm，结合 Fusion 航向输出底盘中心的 `g_odometry.x_mm / y_mm / yaw_deg`。使用公共时间端点、计数历史插值和旋转偏置补偿，标称 500 Hz 更新，待装车确认计数符号与动态精度。参数、流程图和调试方法见 [平面里程计说明](modules/odometry/README.md)。

## 代码分层

| 目录 | 职责 |
| --- | --- |
| `bsp/bsp_g354.*` | SPI3 单字传输、软件片选、复位、TIM2 微秒时基、DRDY 通知 |
| `modules/g354/` | 寄存器初始化与回读、Burst 校验、数据换算，无 HAL/RTOS 依赖 |
| `modules/attitude/` | Fusion 封装、初始倾角、零偏与姿态更新，无 HAL/RTOS 依赖 |
| `modules/fusion/vendor/` | 固定版本的上游源码、MIT 许可证、来源及 SHA-256 清单 |
| `application/imu_app.*` | IMU 任务、静止标定、样本连续性检查、结果发布 |
| `application/imu_config.h` | 标定阈值和安装方向配置 |
| `bsp/bsp_mt6826s.*` | SPI1/PA4、SPI2/PB12 软件片选和片选时序 |
| `modules/mt6826s/` | 编码器连读、CRC、报警和角度换算 |
| `modules/encoder_motion/` | 多圈位移、窗口转速、速度滤波和连续性管理 |
| `application/encoder_app.*` | 两路独立任务、g_encoder/g_encoder2 结果发布和各自错误统计 |
| `application/encoder_config.h` | 编码器采样周期和任务参数 |
| `bsp/bsp_time.*` | IMU、编码器及里程计共用的 TIM2 微秒时基 |
| `modules/odometry/` | 方向矩阵、旋转偏置补偿及平面位姿积分 |
| `application/odometry_app.*` | 采样历史、时间对齐、里程计任务及结果发布 |
| `application/odometry_config.h` | 两轮直径、方向、传动比及滚动直线交点 |

`Core/Src/main.c` 在 USER CODE 区域接入 `imu_app_task()`、`encoder_app_start()` 和 `odometry_app_start()`。
CubeMX 外设初始化保持现状；新增源文件由根目录 CMakeLists.txt 管理。
Fusion 源码保持上游原样。两个 Fusion C 文件在 Debug 下也用 `-O2` 编译，以控制强制内联导致的栈占用和计算时间；其余 Debug 代码保持 `-O0`。

## 接线和外设前提

| STM32 | G354 连接器 |
| --- | --- |
| PC10 / SPI3 SCK | 1 / SCLK |
| PC11 / SPI3 MISO | 2 / SDO |
| PC12 / SPI3 MOSI | 5 / SDI |
| PA15 / G354_CS | 6 / CS，低有效 |
| PD0 / G354_DRDY | 13 / DRDY，高有效 |
| PD1 / G354_RST | 16 / RST，低有效 |
| 3.3 V | 10、11、12 / VCC |
| GND | 3、4、8、15 / GND |

依据 M-G354PDH0 Data Sheet Rev.20160212：SPI 与 UART 仅连接其中一种；未使用的输入按手册通过电阻上拉，17～20 脚不连接。
SPI3 为 Mode 3、16 位、MSB 优先、软件 NSS，目前为 656.25 kHz。
PD0 使用上升沿 EXTI0，中断抢占优先级为 5。TIM2 是 1 MHz、32 位递增计数器，BSP 会启动它。
SPI3 和 IMU 任务的 FreeRTOS direct notification 由 IMU 独占；TIM2 由 `bsp_time` 管理，三路采样共享其连续时间，不得重置或重新配置计数器。

## 采样配置

- 500 Hz，内部采样时钟，移动平均 4 taps。
- `SIG_CTRL=0xFE00`：温度和六轴新数据标志。
- `MSC_CTRL=0x0006`：DRDY 开启、高有效、外部触发关闭。
- `SMPL_CTRL` 高字节为 `0x02`；`FILTER_CTRL` 低字节为 `0x02`。
- `UART_CTRL=0`：关闭 UART Auto 和 Auto Start。
- `BURST_CTRL1=0xF003`、`BURST_CTRL2=0x7000`：FLAG、32 位温度/角速度/加速度、COUNT、CHECKSUM，共 17 个 16 位字；无 UART 帧头/帧尾。
- COUNT 每 500 us 加一，因此 500 Hz 正常相邻帧的差值为 4；使用无符号差值处理 16 位回绕。
- 校验和是前 16 个 16 位字之和的低 16 位。
- 所有配置都在每次启动时写入并回读，未执行 IMU Flash Backup。

SPI 普通访问每字后等待 40 us。Burst 命令后等待 45 us、每个数据字后等待 16 us；在支持的 SCLK（10 kHz～1 MHz）内同时满足手册的 stall 和 read-rate 下限。
实际能否以 500 Hz 完成整包读取仍与 SCLK、调度和负载有关；软件会统计读取耗时并拒绝超过一个采样周期的读取。

## 启动和结果读取

复位低电平至少 100 ms，释放后等待至少 800 ms，再检查就绪位、诊断和产品 ID。
进入采样后先跳过约 100 ms 数据，再积累连续 2 秒静止样本进行零偏与初始重力方向估计。
正常从复位到进入 RUNNING 约需 3 秒；移动、振动、丢帧会重新开始标定，请保持静止。
静止判断无法区分任意小的恒定绕重力轴转动，必须实际固定设备后再启动。

标定要求每轴角速度绝对值不超过 0.5 dps、窗口标准差不超过 0.05 dps；
加速度模长偏差不超过 0.05 g、每轴窗口标准差不超过 0.01 g。
2026-10-28 板上观察到 500 Hz / 4 taps 配置下 X/Y 轴静止标准差约 0.031 dps，
原 0.03 dps 门限导致每满 1000 点便重新标定，因此调整为 0.05 dps。
该参数仍需结合安装振动和实际运动工况评估。

调试器可观察全局结构体 `g_imu`：

| 字段 | 含义 |
| --- | --- |
| `state` | 0 初始化、1 标定、2 运行、3 错误 |
| `valid` | 本次姿态是否可用，需同时检查更新时间 |
| `roll_deg / pitch_deg / yaw_deg` | ZYX 欧拉角，单位度 |
| `quaternion[4]` | `[w,x,y,z]`，机体到世界坐标系的旋转 |
| `gyroscope_dps / accelerometer_g` | 完成安装方向变换后的六轴数据，单位 dps/g |
| `gyroscope_bias_dps` | 当前陀螺仪零偏估计 |
| `calibration_samples / calibration_restarts` | 标定进度和重启次数 |
| `drdy_interrupts / samples / attitude_updates` | 中断、有效数据包、姿态更新计数 |
| `dropped_samples / checksum_errors / timing_errors` | 采样丢失、校验错误、时序异常 |
| `error / sensor_status / diagnostic / spi_error` | 应用错误、驱动状态、传感器诊断、HAL SPI 错误 |
| `max_read_time_us / max_fusion_time_us` | 实测最大读取/算法耗时，TIM2 计量 |

任务中的消费者应通过 `imu_app_get_output()` 获取一致快照；函数还检查数据是否超过 10 ms 未更新。
该接口仅供任务调用。不要直接跨任务读取整个 `g_imu`，也不要在 ISR 中调用它。

```c
#include "imu_app.h"

imu_output_t imu;
if (imu_app_get_output(&imu)) {
    /* 使用 imu.roll_deg、imu.pitch_deg、imu.yaw_deg。 */
}
```

目前通过调试变量和读取接口发布结果，未添加串口输出。

## 坐标、航向和有效性

默认机体坐标轴与手册 Figure 3.1 的传感器坐标轴相同，世界坐标系 +Z 向上。
实际安装不同，请修改 `imu_config.h` 的轴映射，保持右手系；编译时会检查置换和符号。
初始 Roll/Pitch 由重力方向决定，不强制将倾斜安装的角度置零；初始 Yaw 定义为零。
欧拉角 Roll/Yaw 约为 ±180 度，Pitch 约为 ±90 度，在俯仰接近 ±90 度时请使用四元数。
Yaw 是相对启动航向，不具备地理北向含义，也不是多圈累计角；六轴输入无法消除其长期漂移。

Fusion 使用 450 dps 量程、0.5 增益、10 度加速度拒绝阈值和 5 秒恢复时间。
运行时零偏检测设为 0.05 dps、持续 3 秒，并在加速度模长偏离 1 g 超过 0.03 g 时停止学习。
这些是首版参数，需按真实静止噪声和运动工况调整；小于阈值的持续慢转仍可能被误判为零偏。

丢帧、跨周期读取或陀螺仪过量程会让 `tracking_continuous` 保持 false。
软件跳过缺失区间的积分；之后仍输出估计角度供诊断，但 `valid` 保持 false，重启并重新静止标定才能建立新参考。
这能避免把无法重建的运动错误标记为可靠姿态。普通加速度拒绝和暂时恢复也有独立状态字段。
连续 3 次 DRDY 等待超时、连续 10 次读取错误或传感器诊断故障，会进入 ERROR，保留原始错误且不循环复位传感器。

## 构建和离线回归

工程已有 Debug CMake preset：

```powershell
cmake --preset Debug
cmake --build --preset Debug --parallel 4
```

测试将实际 G354/Fusion C 源码交叉编译成 ARM 机器码，再使用 Unicorn 执行，不接触硬件。
需要 Arm GNU 工具链和 Python，安装测试依赖到被忽略的 build 目录即可：

```powershell
python -m pip install --target build/imu-test-tools unicorn
python tests/run_imu_regression.py
```

可通过 `--compiler <arm-none-eabi-gcc完整路径>` 指定编译器。
回归覆盖负数和低 16 位数据换算、温度、损坏校验和、错误标志、寄存器配置与回读失败、Burst 时序、
已知静止倾角、90 度航向转动，以及加速度干扰与过量程恢复。
离线测试不能证明板上接线、中断、电气时序和真实动态精度，烧录后应先观察状态、数据计数和误差计数，再评估角度。

## 上游

- 仓库：https://github.com/xioTechnologies/Fusion
- 固定提交：`a8d7224f36a0ec82345ef49a3db50e65f8d3bab8`
- 许可证：`modules/fusion/vendor/LICENSE.md`（MIT）
- 原始文件 SHA-256：`modules/fusion/vendor/UPSTREAM.json`

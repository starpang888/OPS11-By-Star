# STM32 IMU 与磁编码器

当前为收尾配置：编码器启动读法对比和运行期配置诊断默认关闭，保留启动配置、每帧 CRC/报警、DMA 超时恢复。里程计支持有界的同步等待与恢复，串口协议保持不变。参数清单、状态处理及待装车验收项目见 [工程收尾说明](docs/project_handover.md)。本次收尾代码已通过 Debug 编译，尚未烧录验证。

STM32F407VET6、FreeRTOS、Epson M-G354PDH0，使用论文的自适应角速度 KF、联合静止零偏修正和二阶 Runge-Kutta 四元数积分，输出相对开机姿态的 ZXY 欧拉角。
本版于 2026-10-31 替换 Fusion；固件编译通过，尚未进行本版上板验证。详细参数、公式修正和温漂标定方法见 [姿态模块说明](modules/attitude/README.md)。

已接入两个 MT6826S 编码器，分别使用 SPI1/PA4 和 SPI2/PB12，各自用独立任务进行标称 1000 Hz 单圈角度采样，提供 CRC 校验、报警统计和实际读取频率。
两路已改为 SPI 收发 DMA、软件片选、完成中断通知任务，支持超时及错误恢复；新增 `g_encoder/g_encoder2.dma` 调试统计。DMA 版本编译通过，待上板确认。
用户已反馈双路角度、多圈位移和转速数据正常；本次关闭开发诊断的收尾配置仍待上板确认。详见 [MT6826S 接入说明](modules/mt6826s/README.md)。

已加入第一版双随动轮平面里程计：SPI1 测前后、SPI2 测左右，两轮直径 58 mm，结合 IMU 航向输出底盘中心的 `g_odometry.x_mm / y_mm / yaw_deg`。使用公共时间端点、计数历史插值和旋转偏置补偿，标称 500 Hz 更新，待装车确认计数符号与动态精度。参数、流程图和调试方法见 [平面里程计说明](modules/odometry/README.md)。

## 代码分层

| 目录 | 职责 |
| --- | --- |
| `bsp/bsp_g354.*` | SPI3 单字传输、软件片选、复位、TIM2 微秒时基、DRDY 通知 |
| `modules/g354/` | 寄存器初始化与回读、Burst 校验、数据换算，无 HAL/RTOS 依赖 |
| `modules/attitude/` | 自适应 KF、温漂拟合/补偿、联合静止判据、渐进零偏、RK2，无 HAL/RTOS 依赖 |
| `modules/fusion/vendor/` | 历史 Fusion 源码及许可证，已不参与固件构建 |
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
| `bsp/bsp_telemetry.*` | USART2 DMA 发送、完成/错误回调、超时恢复和缓冲区管理 |
| `modules/odometry_protocol/` | 位姿 ASCII 报文编码和 XOR 校验，无 HAL/RTOS 依赖 |
| `application/telemetry_app.*` | 低优先级任务，标称 100 Hz 获取里程计快照并发送 |
| `application/telemetry_config.h` | 串口输出周期和任务栈配置 |
| `bsp/bsp_oled.*` | I2C3 地址应答、命令/显存数据分块发送 |
| `modules/oled/` | 128×64 OLED 初始化、字形、帧缓存及分页刷新 |
| `application/display_app.*` | 低优先级显示任务，读取里程计快照并显示 X/Y/Yaw |
| `application/display_config.h` | 显示控制芯片、7 位地址、刷新周期和栈配置 |

`Core/Src/main.c` 在 USER CODE 区域接入 `imu_app_task()`、`encoder_app_start()`、`odometry_app_start()`、`telemetry_app_start()` 和 `display_app_start()`。
CubeMX 外设初始化保持现状；新增源文件由根目录 CMakeLists.txt 管理。
Fusion 历史源码保持上游原样，新的姿态模块由根目录 CMakeLists.txt 构建。

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
进入采样后先跳过约 100 ms 数据，再积累连续 2 秒静止样本估计零偏、角速度噪声方差和初始温度；加速度检查标定期间是否稳定。
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
| `roll_deg / pitch_deg / yaw_deg` | ZXY 欧拉角，单位度，相对开机姿态 |
| `quaternion[4]` | `[w,x,y,z]`，机体到初始导航坐标系的旋转 |
| `gyroscope_dps / accelerometer_g` | 完成安装方向变换后的六轴数据，单位 dps/g |
| `gyroscope_bias_dps` | 当前陀螺仪零偏估计 |
| `calibration_samples / calibration_restarts` | 标定进度和重启次数 |
| `drdy_interrupts / samples / attitude_updates` | 中断、有效数据包、姿态更新计数 |
| `dropped_samples / checksum_errors / timing_errors` | 采样丢失、校验错误、时序异常 |
| `error / sensor_status / diagnostic / spi_error` | 应用错误、驱动状态、传感器诊断、HAL SPI 错误 |
| `max_read_time_us / max_fusion_time_us` | 实测最大读取/算法耗时，TIM2 计量 |
| `stationary / stationary_block_mask / bias_updates` | 联合静止状态、阻塞原因、渐进零偏更新次数 |
| `compensated_gyroscope_dps / corrected_gyroscope_dps` | 死区前、死区后角速度，dps |
| `temperature_compensated / temperature_calibration` | 温度模型启用状态、实测温漂标定进度与拟合结果 |

任务中的消费者应通过 `imu_app_get_output()` 获取一致快照；函数还检查数据是否超过 10 ms 未更新。
该接口仅供任务调用。不要直接跨任务读取整个 `g_imu`，也不要在 ISR 中调用它。

```c
#include "imu_app.h"

imu_output_t imu;
if (imu_app_get_output(&imu)) {
    /* 使用 imu.roll_deg、imu.pitch_deg、imu.yaw_deg。 */
}
```

IMU 通过调试变量和读取接口发布；平面里程计另通过 USART2 输出，见下节。

## USART2 位姿输出

当前配置：PD5=TX、PD6=RX，115200、8N1、无流控。
TX 使用 DMA1 Stream6 / Channel4、Normal、Memory increment、Byte，DMA 和 USART2 全局中断抢占优先级均为 6。
串口任务优先级 Low，默认每 10 ms 发送一帧（标称 100 Hz）；实际频率受任务调度和链路忙碌影响。
本版只发送位姿，尚无接收命令。USART2 和它的 HAL UART 回调由本 BSP 管理，请勿另外通过 printf 或其他任务占用。

上板连接：PD5 接 USB 转 TTL 的 RX，GND 共地，使用 3.3 V TTL 电平。
若以后接收指令，再将转接器 TX 接 PD6；当前只看输出时 PD6 可以不接。
串口助手设置 115200、8 数据位、无校验、1 停止位、无流控，使用文本显示。

每帧格式：

```text
$ODOM,seq,t_us,x_mm,y_mm,yaw_deg,valid,continuous,state,error*HH\r\n
```

这里 `\r\n` 表示实际的回车和换行字符。例如：

```text
$ODOM,1,123456,0.000,0.000,0.000,1,1,1,0*3C
```

| 字段 | 含义 |
| --- | --- |
| `seq` | 从 0 开始的打包序号，uint32；发送失败可能留下序号空缺，溢出回绕 |
| `t_us` | 里程计采样端点 TIM2 时间戳，微秒，uint32，约 71.6 分钟回绕；不是主机时间或发送时间 |
| `x_mm / y_mm` | 底盘中心相对初始原点的坐标，毫米，三位小数 |
| `yaw_deg` | 相对初始航向，度，范围约 [-180,180]，逆时针为正 |
| `valid` | 本帧位姿有效；还包含快照新鲜度（小于 10 ms）及正常运行状态检查 |
| `continuous` | 从当前里程计原点起是否保持连续跟踪 |
| `state` | 0 启动等待或短时同步等待、1 正常、2 跟踪丢失、3 错误 |
| `error` | 0 无错误、1 任务、2 配置、3 时基、4 输入、5 同步、6 几何 |
| `HH` | 从 `$` 后第一字节到 `*` 前最后字节逐字节 XOR，两个大写十六进制字符 |

启动标定、等待传感器、失去连续性或坐标失效时继续发送 `valid=0` 的状态帧；坐标可能是保留值，接收端仅在 `valid=1` 且校验通过时使用位姿。
接收端也应自行检测串口断流和时间戳是否持续更新；小数位数不代表实测精度。
有限的坐标编码范围为 ±999999999.999 mm；非有限数或超出范围会拒绝打包，并增加 `format_errors`。

DMA 使用独立静态 SRAM 缓冲区，发送完成前不会覆盖。遇到忙碌跳过当次周期，之后发送最新位姿，不排队积压旧数据。
完成计数由 UART TC 中断确认；出错或超过 100 ms 无完成事件时，在任务中终止传输并恢复。
若 DMA 无法确认停止，锁定发送并报告传输错误，避免复用尚在读取的缓冲区。异常恢复可能截断当时的一帧，接收端从下一个 `$` 重新同步并检查校验。

烧录后观察 `g_telemetry`：

| 字段 | 正常预期 |
| --- | --- |
| `running / error` | 1 / 0；error=1 任务创建失败、2 配置不符、3 传输恢复失败 |
| `sent / tx.completed` | 持续增加，正常相差至多约一帧 |
| `pose_valid` | 里程计正常建立原点后为 1 |
| `tx.errors / tx.timeouts / format_errors` | 保持 0 |
| `skipped_busy` | 链路忙碌跳过的发送周期数，可用于观察调度/带宽 |
| `tx.last_hal_error / tx.recoveries` | 最近 HAL 错误和恢复次数 |

USART2 已有上板文本输出；本次收尾配置仍待烧录确认。同步恢复等待期间 `valid=0,continuous=1,state=0,error=5`，重新取得支持的连续端点后恢复 `1,1,1,0`。真正丢失连续性后维持 `valid=0,continuous=0,state=2`，不会自动归零续接。

## 坐标、航向和有效性

已加入用户确认的 SSD1306 128×64 位姿显示，使用 I2C3、PA8=SCL、PC9=SDA、100 kHz，每 500 ms 刷新 X/Y（mm）、Yaw（deg）和定位状态。定位无效时数值显示 `--`。接线和 `g_display` 调试字段见 [OLED 接入说明](modules/oled/README.md)。本次已编译，尚未进行屏幕上板验证。

默认机体坐标轴与手册 Figure 3.1 的传感器坐标轴相同，导航坐标系与开机时机体系重合；平面定位需水平安装，使 +Z 向上。
实际安装不同，请修改 `imu_config.h` 的轴映射，保持右手系；编译时会检查置换和符号。
初始四元数为单位四元数，三轴初始均为零，不使用加速度计持续纠正 Roll/Pitch。
ZXY 的 Roll 范围约为 ±90 度，Pitch/Yaw 约为 ±180 度；Roll 接近 ±90 度时请使用四元数。
Yaw 是相对启动航向，不具备地理北向含义，也不是多圈累计角；六轴输入无法消除其长期漂移。

使用 450 dps 过量程检查。运行时静止检测采用死区前 0.04 dps 门限、连续约 100 ms、加速度模长误差小于 0.03 g，并要求两轮健康且静止。
确认静止后以 0.02 dps/s 的限速更新剩余零偏。按用户选择，当前三轴死区 `IMU_GYRO_DEADBAND_DPS=0.3f`，每轴低于 0.3 dps 的角速度归零；这是暂用门限，之后有转台再标定。论文参考值为 0.15 dps。
温漂系数需要本机实测，默认禁用温度模型。通过 `g_imu_command` 可启动采集、拟合和应用；步骤见姿态模块说明，标定点暂存 RAM，经确认的系数写入配置后重新编译保存。

丢帧、跨周期读取或陀螺仪过量程会让 `tracking_continuous` 保持 false。
软件跳过缺失区间的积分并清除滤波/静止历史，`valid` 保持 false，重启并重新静止标定才能建立新参考。
`accelerometer_ignored=1` 为当前陀螺传播算法的正常输出，`recovering=0`，不再表示 Fusion 的恢复过程。
连续 3 次 DRDY 等待超时、连续 10 次读取错误或传感器诊断故障，会进入 ERROR，保留原始错误且不循环复位传感器。

## 构建

工程已有 Debug CMake preset：

```powershell
cmake --preset Debug
cmake --build --preset Debug --parallel 4
```

本版仅编译和源码复核，未执行测试或烧录。`tests/imu_regression.*` 是历史 Fusion 版本的回归代码，与当前姿态接口不兼容，不代表当前 KF/RK2 算法经过回归。上板后先观察健康状态、连续性、联合静止及零偏更新，再评估静止漂移和正反方向实际转角。

## 历史 Fusion 上游

- 仓库：https://github.com/xioTechnologies/Fusion
- 固定提交：`a8d7224f36a0ec82345ef49a3db50e65f8d3bab8`
- 许可证：`modules/fusion/vendor/LICENSE.md`（MIT）
- 原始文件 SHA-256：`modules/fusion/vendor/UPSTREAM.json`

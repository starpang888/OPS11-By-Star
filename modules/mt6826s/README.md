# MT6826S 双编码器接入

依据 MagnTek《MT6826S 数据手册》中文版 Rev.1.1（2024-01）第 24~29、33、35、40 页。
第一路已在板上读取到角度，普通寄存器读和连续读一致，CRC、格式和 IO 错误均为零。
用户反馈第一路 BW 从 0 改为 5 后，正反向手拨快转的报警计数和角度波形均正常。
用户已反馈两路角度、多圈位移和转速正常。两路均设为 1 kHz；当前收尾配置关闭开发自检和运行期配置诊断，待重新烧录确认。
动态角度精度和 SPI 电气波形仍需验证。

## 分层

| 文件 | 职责 |
| --- | --- |
| `bsp/bsp_mt6826s.c/.h` | SPI1/PA4 和 SPI2/PB12 两组独立上下文、片选时序、对应 HAL 错误读取 |
| `modules/mt6826s/mt6826s.c/.h` | 读取命令、BW 读改写及回读、CRC、固定格式检查、15 位角度换算，无 HAL/RTOS 依赖 |
| `modules/encoder_motion/encoder_motion.c/.h` | 角度展开、整圈数、位移累计、窗口转速和低通滤波，无 HAL/RTOS 依赖 |
| `application/encoder_app.c/.h` | 独立任务、采样调度、错误计数、结果发布和一致快照 |
| `application/encoder_config.h` | 采样周期、启动等待、数据过期时间、任务栈大小 |

每路由自己的任务独占 SPI 和片选，两路复用同一份驱动与任务函数，各自保存输出、计数和诊断状态。
第一路为 `g_encoder`，第二路为 `g_encoder2`；下文以 `g_encoder` 列出的字段在 `g_encoder2` 中也都有。
其他任务通过 `encoder_app_get_output_for(ENCODER_1/ENCODER_2, &output)` 读取数据，原 `encoder_app_get_output()` 继续返回第一路。
该接口供任务上下文调用；不要在 ISR 中调用。
`encoder_app_start()` 在 `main.c` 的 `USER CODE BEGIN RTOS_THREADS` 中、调度器启动前调用。
两个任务通过 CMSIS-RTOS v1 创建，分别名为 `encoderTask`、`encoder2Task`，均为 Normal 优先级、各 512 words（2048 字节）栈，低于现有 IMU 任务优先级。
`encoder_app_start()` 返回两路任务是否全部创建成功；若仅一路失败，另一路继续运行，失败路的 `error=1`。
一路 BSP 配置失败时只停留在该路错误状态；另一路照常运行。SPI 超时或调度抢占仍可能影响共享 CPU 的实际采样频率。
CubeMX 无需另建同名任务。

## 接线和配置

| 信号 | 第一路 SPI1 | 第二路 SPI2 |
| --- | --- | --- |
| SCK | PA5 | PB13 |
| MISO | PA6 | PB14 |
| MOSI | PA7 | PB15 |
| CSN，低有效 | PA4 / MT6826S_CS | PB12 / MT6826S_2_CS |

SPI1：全双工主机、Mode 3（CPOL=1/CPHA=1）、8 位、MSB、软件 NSS、16 分频。
APB2=84 MHz 时，SCK=5.25 MHz。
SPI2：相同协议参数、8 分频，APB1=42 MHz 时，SCK 同为 5.25 MHz。
两路使用硬件 SPI 收发 DMA，Normal 模式；SPI 全局中断用于错误处理，软件计算 CRC。PA4 和 PB12 为普通推挽输出，启动默认高电平，不启用硬件 NSS 输出。
BSP 检查 CubeMX 已生成的 SPI、DMA、NVIC 和片选配置，然后使能 SPI，建立空闲高电平时钟。检查失败时对应编码器 `error=ENCODER_ERROR_BSP(2)`。

| 请求 | DMA / Stream / Channel | 方向 |
| --- | --- | --- |
| SPI1_RX | DMA2 Stream0 Channel3 | Peripheral to Memory |
| SPI1_TX | DMA2 Stream3 Channel3 | Memory to Peripheral |
| SPI2_RX | DMA1 Stream3 Channel0 | Peripheral to Memory |
| SPI2_TX | DMA1 Stream4 Channel0 | Memory to Peripheral |

四路均为 Normal、Byte/Byte、外设地址固定、内存地址递增、High 优先级、FIFO Disabled。DMA 中断抢占优先级为 6，当前 SPI1/SPI2 全局中断为 5，均符合 FreeRTOS 中断接口数值 >=5 的要求。`MX_DMA_Init()` 在 SPI 初始化前执行，生成的 IRQ 调用对应 HAL 处理函数。

## 读取过程

启动等待 10 ms 加一个 RTOS tick，随后按 1 ms 周期（标称 1000 Hz）读取：

1. CSN 拉低，芯片锁存当前角度。
2. 同一片选内通过 `HAL_SPI_TransmitReceive_DMA()` 发送 `A0 03 00 00 00 00`，同步接收六字节。任务等待完成通知，让出 CPU。
3. HAL RX DMA 完成流程确认 SPI 传输结束，进入 BSP 回调，保持片选保护时间后将 CSN 拉高，并通知对应任务；任务提取最后四字节。失败路径同样释放片选。
4. 对寄存器 0x003~0x005 的三个字节计算 CRC，与 0x006 比较。
5. 检查固定零位、三种报警，再发布正常角度。

角度位于 `0x003[7:0]` 和 `0x004[7:1]`：

```c
angle_raw = (data[0] << 7) | (data[1] >> 1);
angle_deg = angle_raw * 360.0f / 32768.0f;
```

CRC 使用多项式 `0x07`，MSB 先行，初值 `0x00`，无反射、无最终异或。
手册给出了多项式和移位方向，但未明确初值与最终异或；这两项首版按零实现，需用板上数据确认。
如果持续 CRC 错误，应对照 `raw_frame`、`crc_received` 和 `crc_calculated` 定位，不能跳过校验。

`angle_deg` 是单圈绝对角度，范围 `[0,360)`，最小步进约 0.010986 度；分辨率不代表实际精度。
数值保留芯片当前零位和方向配置，不在启动时清零。另通过 `motion` 输出相对启动位置的多圈位移与转速，详见下文。
周期角度读取只使用读命令。另有启动时的临时 BW 修改，详见下文；不触发自校准或 EEPROM 烧录。

## DMA 实现与调试

协议模块仍通过同步返回的 `bus.transfer()` 抽象访问总线，BSP 内部为 DMA + FreeRTOS 任务通知等待，不再逐字节轮询接收。启动配置读取、可选的普通寄存器核对及 BW 读写同样走 DMA，只有 SPI1/SPI2 使用此机制；SPI3/G354 保持现有 DRDY 通知和读取实现。

每路有独立、4 字节对齐的静态 TX/RX 缓冲区，位于 DMA 可访问的普通 SRAM；不要将其迁移到 CCMRAM。任务先拷贝命令，启动 DMA 后在自己的 direct notification 上等待最多 2 个 RTOS tick（当前 tick=1 ms，实际恢复还受任务调度影响）。只有短暂的 HAL 启动和完成状态交接使用临界区，线路传输和任务等待期间不屏蔽中断；关闭无用的半传输中断。

超时或 DMA/SPI 错误时，本包作为 IO 错误返回，先停止请求和回调，再分别中止 RX/TX DMA、恢复 SPI 并释放 CS，清除遗留通知。不能确认 DMA 已停或恢复失败时，本路拒绝继续启动，防止复用仍被 DMA 访问的缓冲区。没有自动重读替代坏帧，里程计的丢失检测继续生效。SPI/DMA 全局回调由本 BSP 分发，之后其他 SPI 采用 DMA 时应在这里扩展分发，不能重复定义同名回调。编码器任务的 direct notification 不可同时用于其他功能。

在 `g_encoder.dma` 和 `g_encoder2.dma` 中观察：

| 字段 | 含义与预期 |
| --- | --- |
| `started / completed` | 启动次数 / 成功收发完成次数，持续增加，包含诊断访问 |
| `start_errors` | HAL 启动失败，预期 0 |
| `transfer_errors` | DMA 或 SPI 完成错误，预期 0 |
| `timeouts` | 等待超时，预期 0 |
| `recovery_errors` | DMA/SPI 无法恢复，预期 0 |
| `last_error` | 最近一次 BSP 记录的 HAL SPI 错误，成功后保留，初值 0 |
| `max_completion_us` | 从 CS 拉低到完成回调的最大延迟，包含调度与中断延迟 |

`completed` 只表示 SPI/DMA 完成，不代表编码器 CRC 或状态健康；仍需结合 `crc_errors / alarm_samples / valid`。保持每路标称 1 kHz，观察 `measured_read_rate_hz` 确认实际速率；里程计继续标称 500 Hz。当前版本编译通过，DMA 上板频率及连续运行需要烧录后确认。

## 插件观察变量

| 变量 | 含义 |
| --- | --- |
| `g_encoder.angle_deg` | 最近一次健康读数，单位度，可添加到波形图 |
| `g_encoder.angle_raw` | 原始 15 位角度，0~32767 |
| `g_encoder.state` | 0 初始化、1 正常运行、2 错误 |
| `g_encoder.error` | 0 无错误、1 任务创建失败、2 BSP 配置失败、3 传感器读取失败、4 运动计算参数错误 |
| `g_encoder.sensor_status` | 0 正常、1 参数、2 IO、3 CRC、4 格式、5 芯片报警 |
| `g_encoder.valid` | 最近一次尝试是否得到正常数据；消费者还需通过 getter 检查数据年龄 |
| `g_encoder.timestamp_ms` | 最近一次健康采样开始时间，HAL 毫秒计数，可回绕 |
| `g_encoder.attempts / samples` | 尝试次数 / 健康样本数 |
| `g_encoder.sample_rate_hz` | 设定采样频率，当前为 1000 Hz |
| `g_encoder.measured_read_rate_hz` | 约每秒更新一次的实际周期读取频率，含失败尝试，不含额外诊断读取；首个统计窗口完成前为 0 |
| `g_encoder.crc_errors / format_errors / io_errors` | 累计错误次数 |
| `g_encoder.alarm_samples / consecutive_errors` | 报警样本数 / 连续失败次数 |
| `g_encoder.overspeed / weak_magnet / undervoltage` | 超速、弱磁、欠压报警 |
| `g_encoder.crc_valid` | 最近一次尝试的 CRC 是否通过 |
| `g_encoder.raw_frame[4]` | 最近一次完整接收的寄存器原始字节；IO 失败时保留上次值 |
| `g_encoder.crc_received / crc_calculated` | 收到的 CRC / 软件计算值 |
| `g_encoder.spi_error` | HAL SPI 错误码 |
| `g_encoder.overspeed_samples / weak_magnet_samples / undervoltage_samples` | 各类报警累计帧数；同一帧可增加多项 |
| `g_encoder.last_alarm_bits / last_alarm_frame[4]` | 最近一次 CRC 和格式均通过的报警码与原始帧，恢复正常后保留 |
| `g_encoder.last_alarm_angle_deg / last_alarm_ms` | 报警帧中的角度和采样时间，仅用于诊断 |
| `g_encoder.diagnostics.bandwidth` | 芯片系统带宽 BW（0~7），手册出厂默认值为 5 |
| `g_encoder.diagnostics.bandwidth_register` | 0x011 原始值，包含保留位 |
| `g_encoder.diagnostics.calibration_state` | 0 未运行、1 正在校准、2 校准失败、3 校准成功 |
| `g_encoder.diagnostics.calibration_control` | 0x155 原始值，0x5E 表示软件使能自校准 |
| `g_encoder.diagnostics_read_ok / diagnostics_status` | 最近一组诊断寄存器读取的传输结果 |
| `g_encoder.diagnostics_timestamp_ms / diagnostics_io_errors` | 最近成功读取时间和诊断读取失败次数 |
| `g_encoder.register_check.single_angle_deg / burst_angle_deg` | 普通寄存器读 / 连读取得的角度，用于静止时交叉核对 |
| `g_encoder.register_check.single_crc_valid / burst_crc_valid` | 两条读取路径各自的 CRC 和固定格式是否通过 |
| `g_encoder.register_check.single_status / burst_status` | 两条路径的驱动状态码（与 sensor_status 编码相同） |
| `g_encoder.register_check.single_frame[4] / burst_frame[4]` | 两条路径读取的原始角度、状态和 CRC |
| `g_encoder.register_check.angle_difference_deg` | 两次读数的最短有符号角差，仅在两条路径校验通过时有意义 |
| `g_encoder.register_check.nonzero_pair / passed` | 是否排除全零响应 / 本次静止对比是否符合 1 度容差 |
| `g_encoder.register_check.checks / passes / timestamp_ms` | 对比次数 / 通过次数 / 最近对比完成时间 |
| `g_encoder.bandwidth_change_evaluated` | 本次 MCU 启动是否执行带宽设置步骤；启动配置读取成功后尝试一次 |
| `g_encoder.bandwidth_change_status` | 设置步骤结果：0 成功、2 IO 错误、6 初始 BW 不符合预期或回读不符；仅 evaluated=true 时有效 |
| `g_encoder.bandwidth_change.before_register / target_register / after_register` | 0x011 修改前 / 目标 / 回读原始值，包含保留位 |
| `g_encoder.bandwidth_change.before_read_ok / after_read_ok` | 修改前读取 / 修改后读取是否传输成功；已经为目标档时复用初始读数 |
| `g_encoder.bandwidth_change.write_attempted / verified` | 是否发起过写入 / 整字节是否符合目标；verified 不代表快转报警已解决 |
| `g_encoder.motion.total_angle_deg` | 相对第一个健康位置的累计角度，double，单位度，可正负、跨圈连续 |
| `g_encoder.motion.total_counts` | int64 精确累计位移，一圈 32768 counts |
| `g_encoder.motion.turns` | int64 净位移的完整圈数，向零取整；+540 度为 1，-540 度为 -1，不是经过芯片零点的次数 |
| `g_encoder.motion.speed_raw_rpm` | 约 10 ms 窗口的平均转速，未低通滤波；正值表示原始单圈角度递增方向 |
| `g_encoder.motion.speed_rpm` | 20 ms 时间常数的一阶低通转速，RPM，正负表示方向 |
| `g_encoder.motion.initialized / valid / tracking_continuous` | 是否建立起点 / 本次累计位移是否有效 / 相对起点的连续性是否一直保留 |
| `g_encoder.motion.speed_valid / speed_updates` | 是否完成连续健康数据的速度窗口 / 转速更新次数 |
| `g_encoder.motion.discontinuities` | 丢失的连续片段数；持续错误不会每帧重复增加 |

报警字段仅在当前帧 CRC 和格式通过时更新为相应报警；CRC/IO 失败时清除，不能据此认定芯片无报警。
默认只在启动时读取 0x011、0x113、0x155，运行期不再周期读取配置寄存器。`ENCODER_RUNTIME_DIAGNOSTICS_ENABLED=0` 是当前收尾配置；开发排查时可改为 1，此时每秒启动一组诊断，三次读取分散到三个采样周期，每周期最多追加一次传输，全部成功后发布结果。BW 设置只在启动诊断阶段尝试，方法在下文单独说明。
单寄存器读取协议没有 CRC，`diagnostics_read_ok` 仅代表 HAL 传输成功，不能证明寄存器值正确；
三项顺序读取也不是原子快照。读取失败保留上次诊断值，通过状态和时间识别是否过期。
普通读取与连读对比仅供开发，默认 `ENCODER_STARTUP_REGISTER_CHECK_ENABLED=0`，`register_check` 各字段保留为 0 不代表故障。需要排查协议时改为 1 并重新编译：启动诊断后按手册图 24 使用四个普通读命令读取 0x003~0x006，再额外连读一次角度。此对比仅执行一次，进入周期采样后不会重复，`checks/passes` 不会持续增加。
普通读数据也包含 CRC，可检查普通命令读取路径；两次角度采用圆周差值比较，容差为 1 度。
`register_check.passed` 还要求两组数据各自不全为零，避免把悬空或不响应产生的全零数据当成通信证据。
此对比不要求两帧属于同一采样时刻；只在轮子静止时判断，快速运动时角差变大不代表通信异常。
报警帧可用于通信诊断，但 `passed=true` 不代表该帧角度可用于控制，也不能证明 BW 寄存器地址必然正确。
`calibration_control` 只能反映软件控制，CAL_EN 引脚也可独立开启校准；不用校准时需按手册保持引脚低电平。
STATUS[0] 在手册中定义为超过 120,000 RPM 的超速报警；手拨时出现该位不能直接认定为真实超速，
应结合芯片实际型号、BW、自校准状态以及磁铁安装情况继续排查。
发生 CRC、格式、通信错误或芯片报警时，不更新角度和健康采样时间，`valid=false`。
每个周期继续读取，正常帧到达后自动恢复；BSP 配置错误和任务创建错误需要修正配置后重启。
`encoder_app_get_output()` 通过临界区复制整个结构体，并将超过 10 ms 的样本标为无效。
直接观察全局结构体是非原子读取，也不会在任务停止运行后自动清除 `valid`。

```c
#include "encoder_app.h"

encoder_output_t encoder;
if (encoder_app_get_output(&encoder)) {
    /* 使用 encoder.angle_deg。 */
}

encoder_output_t encoder2;
if (encoder_app_get_output_for(ENCODER_2, &encoder2)) {
    /* 使用 encoder2.angle_deg。 */
}
```

## 板上确认

烧录当前 ELF 后，先观察 `state=1`、`valid=true`、`samples` 持续增加，错误计数是否保持零。
缓慢转动磁铁，检查角度是否变化，以及跨越 360/0 度时的正常单圈跳变。
若磁场过弱或电压不足，正常校验的状态帧会置位对应报警并使数据无效。

### 核对 BW=0 的读数

1. 开发排查时先设置 `ENCODER_STARTUP_REGISTER_CHECK_ENABLED=1`，重新编译烧录，轮子停在明显非零的角度，等待至少两秒。
2. 查看 `g_encoder.register_check.passed` 是否为 true，两条路径 CRC 是否通过，两个角度是否相近。
3. 当前运行版本只在启动时执行该核对；如需在另一角度复核，请停稳后重启。两条路径应跟随启动位置变化，`passes` 保留本次启动结果。
4. 如果普通读取始终为零而连读角度正常，或普通读 CRC 失败，先排查普通读协议和实物型号，不能据此修改 BW。
5. 板上目前已通过该核对，继续进行下方 BW 对照。

### 临时将 BW=0 改为 5

`application/encoder_config.h` 中 `ENCODER_BANDWIDTH_CHANGE_ENABLED=1` 启用本次对照，目标 `ENCODER_BANDWIDTH_TARGET=5`。
每次 MCU 启动，每路在配置寄存器读取成功后执行一次设置；若显式启用了开发读法对比，还要求对比通过：

1. 重新读取 0x011，若 BW=0，以 `(old & 0xF8) | 5` 保留 Z_EDGE 和保留位。
2. 使用 `60 11 data` 写运行寄存器，随后回读，要求整字节等于目标。
3. 若已为 BW=5，直接确认，不再写；若为其他档位或出现读写错误，记录结果，本次启动不重试。

设置结果独立于周期角度读取状态，需单独观察 `bandwidth_change_status` 和 `verified`。
没有 EEPROM 烧录命令，编码器断电重启后会重新加载 EEPROM 配置；这版固件会再次执行上述步骤。
仅复位 MCU 不一定会复位编码器，因而下一次可能看到 `before_register` 已为 5、`write_attempted=false`。
关闭配置宏只会停止之后的自动修改，不会把已供电编码器的 BW 自动还原为 0。

烧录后将轮子停在非零角度，等待至少两秒，确认：

- `bandwidth_change_evaluated=1`、`bandwidth_change_status=0`、`bandwidth_change.verified=1`。
- `diagnostics.bandwidth=5`，修改前后原始寄存器值除低三位外相同。
- `crc_errors / format_errors / io_errors` 不增加。

记录 `alarm_samples` 和 `overspeed_samples` 的起始值，再按之前相似力度快拨轮子，比较计数增量和波形。
累计计数不会因 BW 修改清零；BW 回读正确只能确认设置生效，报警原因仍需以对照结果判断。

### 提高读取频率

采样周期已从 2 ms 改为 1 ms，标称频率从 500 Hz 提高到 1000 Hz。
SPI1 和 SPI2 均为 5.25 MHz，每路每次角度传输 6 字节，纯时钟时间约 9.14 us。
当前 FreeRTOS tick 为 1 kHz，1 ms 是现有按 tick 延时调度的最小周期；实际读取频率受 IMU 高优先级任务及诊断读取影响。
通过 `measured_read_rate_hz` 观察实际速度，该统计包含周期读取失败，不代表健康数据率。
用户已反馈第一路 BW=5 后正反向快转、报警和通信错误计数及角度波形均正常。

### 第二路板上确认

烧录双路版本后，将两个轮子分别转至非零角度并静止至少两秒，再分别观察 `g_encoder` 和 `g_encoder2`：

- `state=1`、`error=0`、`valid=true`、`samples` 持续增加。
- `sample_rate_hz=1000`；实际频率以各路 `measured_read_rate_hz` 为准，两路采样时刻并不严格同步。
- `diagnostics.bandwidth=5`、`bandwidth_change.verified=true`。默认模式不要求初始角度非零；只有启用开发读法对比时，零角度可能导致该核对不通过并跳过本次启动的 BW 设置。
- CRC、格式、IO 和报警计数不增加，分别转动时各自的 `angle_deg` 跟随对应轮子。

诊断、读路径核对和 BW 设置结果均单独保存，第二路不会使用第一路的核对结果来允许写入。

### 多圈位移和转速

每路第一个健康周期样本作为软件位移零点，`motion.total_angle_deg=0`，硬件单圈角度保持原值。
相邻健康样本的 15 位原始差值映射到 `(-16384,16384)`，再累加到 int64 计数器，处理 360/0 度跨界。
精确半圈的差值无法判断方向，会中断连续性；实际位移超过半圈可能混叠成错误方向，无法仅靠两个角度检出。
因此必须保证每两个有效采样之间实际转动小于半圈；理想 1 ms 间隔对应严格小于 30000 RPM，
这只是采样展开的数学边界，并不是经过板上验证的工作转速。间隔变长时允许的无歧义转速也会下降。

速度每积累至少 10 ms 健康数据更新一次，以窗口内净位移除以实际毫秒间隔换算 RPM，
使用 BSP 在周期角度片选拉低附近记录的 HAL tick，而不是假设每次调度正好相隔 1 ms。
毫秒时间戳存在量化及抢占误差，窗口平均降低逐点差分的噪声；两路角度仍按标称 1 kHz 读取。
`speed_raw_rpm` 为窗口平均，`speed_rpm` 再经过一阶低通，系数 `dt / (20 ms + dt)`，启动及恢复时从零开始滤波。
这会引入一定滞后；停止后原始窗口速度归零，滤波速度逐渐接近零，不使用可能吞掉慢速运动的死区。
当原始窗口转速为零，且滤波尾数的绝对值小于 `FLT_MIN`（约 1.17549e-38 RPM）时，将尾数清为精确零，
避免浮点舍入令滤波残余停留在 1.4013e-45 等非正规数上；这不是机械静止判断阈值。
相关参数位于 `application/encoder_config.h`：`ENCODER_MOTION_MAX_GAP_MS=5`、`ENCODER_SPEED_WINDOW_MS=10`、`ENCODER_SPEED_FILTER_TAU_MS=20`。

CRC、格式、IO 错误、芯片报警，或相邻健康采样间隔超过 5 ms，都会使 `motion.tracking_continuous=false` 并锁存。
不跨未知区间积分，保留已经累计的位移；恢复后只追加新连续片段，`motion.valid` 仍保持 false。
速度在新的连续健康窗口完成后可独立恢复，必须检查 `motion.speed_valid`。
上层 `valid` 继续表示单圈角度是否健康，不能替代多圈位移或速度各自的有效标志。
getter 会同时检查样本是否过期；直接看全局变量时，任务停止后的标志不会自行刷新。

若需重新建立软件位移起点，可从任务上下文请求：

```c
(void)encoder_app_reset_motion(ENCODER_1); /* 或 ENCODER_2。 */
```

请求由对应采样任务处理，在之后的第一个健康位置重新归零位移，并清除运动连续性、速度历史和运动计数。
函数返回 true 只代表请求已排入，不表示已经完成；不会清除 SPI 错误统计或修改芯片硬件零位。
复位 MCU 也会重建位移起点；断电后不能恢复之前累计的圈数。

烧录后分别观察两路：

1. 启动静止，累计角度接近零，`motion.valid=true`；至少一个速度窗口后 `speed_valid=true`，转速接近零。
2. 沿单圈角度递增方向转一圈，累计角度增加约 360 度；反转一圈回到约零，速度符号随方向改变。
3. 穿过单圈 0/360 度边界，累计角度应连续；同时两路角度、速度和错误统计应各自对应自己的轮子。
4. 保持正常采样时 `motion.discontinuities=0`，实际采样频率参考各路 `measured_read_rate_hz`。

该芯片角度读取没有产品 ID 应答；全零帧也满足零初值 CRC，因此不能只靠全零读数证明连接成功。
应实际转动磁铁确认数据响应。若 MISO 断开后悬空，读数行为不可靠。

# 工程收尾说明

当前软件包含 G354 姿态、双 MT6826S 随动轮、底盘中心平面里程计和 USART2 位姿输出。2026-10-31 收尾修改通过 Debug 编译；本次未运行测试、未烧录、未操作硬件。已有双编码器和串口上板反馈不等于本次恢复逻辑已通过实测。

## 固定配置与入口

| 项目 | 当前配置 | 修改位置 |
| --- | --- | --- |
| IMU | G354，SPI3，DRDY，500 Hz，论文 KF/RK2 | `application/imu_config.h`、`modules/g354/` |
| 角速度死区 | 三轴 0.3 °/s，暂用值 | `application/imu_config.h` |
| 温漂模型 | 默认关闭，需本机标定 | `application/imu_config.h` |
| 前后轮 / 左右轮 | SPI1/PA4、SPI2/PB12，各 1 kHz DMA | `application/encoder_config.h` |
| 编码器 BW | 启动确认并设置为 5，不烧写 EEPROM | `application/encoder_config.h` |
| 启动读法对比 | 默认 0，开发排查时才启用 | `ENCODER_STARTUP_REGISTER_CHECK_ENABLED` |
| 运行期配置诊断 | 默认 0，正常采样无额外配置读取 | `ENCODER_RUNTIME_DIAGNOSTICS_ENABLED` |
| 两轮几何 | 直径 58 mm，方向 0°/90°，方向符号 +1，传动比 1 | `application/odometry_config.h` |
| 滚动直线交点 | 相对底盘中心 `cx=-20 mm,cy=+20 mm` | `application/odometry_config.h` |
| 位姿更新 / 发送 | 标称 500 Hz / 100 Hz | `application/odometry_app.c`、`application/telemetry_config.h` |
| 串口 | USART2，PD5 TX、PD6 RX，115200、8N1，无流控，TX DMA | `LED.ioc`，由 CubeMX 生成 |
| OLED | 已确认 SSD1306 128×64；I2C3 PA8=SCL、PC9=SDA，100 kHz，500 ms 刷新 | `application/display_config.h` |

默认 `register_check` 为 0 是关闭开发功能的结果。`diagnostics_timestamp_ms` 保留启动配置读取时间，不再每秒刷新。每帧 STATUS 报警和 CRC 仍持续检查。详细调试字段保留，方便现有 EmberProbe 观察；没有增加新的监测任务。

代码继续采用三层：BSP 管总线、DMA 与时钟，modules 管协议与数学，application 管采样、同步、发送和显示。任务优先级：IMU AboveNormal > 两路编码器 Normal > 里程计 BelowNormal > 串口/OLED Low。显示使用分块 I2C 轮询，不屏蔽采样中断，每块后让出 CPU。CubeMX 生成区域没有手改；应用入口位于 `main.c` 的 USER CODE 区域。OLED 接线和型号配置见 [OLED 接入说明](../modules/oled/README.md)，本次新增显示尚未上板确认。

## 异常后的行为

| 情况 | 行为 |
| --- | --- |
| 启动标定 / 等待原点 | 保持静止，发送无效状态，等待首个健康公共端点 |
| 等待编码器后侧样本 | 等待实际样本，不外推 |
| 某个插值跨度 >4 ms 或端点延迟 ≥8 ms | 跳过该端点；暂时无效，保留连续性，尝试后续公共端点 |
| 后续端点连续、跨度合格、距上次成功端点 ≤6 ms | 用累计计数差和连续航向继续积分，保留原点；累计一次恢复 |
| 无法恢复、可用端点间隔 >6 ms，或输出持续过期 ≥10 ms | 锁存跟踪丢失，坐标保留且无效 |
| 编码器 CRC/报警/计数中断或重置，IMU 丢帧/过量程 | 不使用坏数据，定位连续性失效，不自动续接未知运动 |
| UART 忙碌 | 跳过发送周期，之后发送最新位姿，不积压 |
| UART DMA 出错或发送超时 | 在任务中停止并恢复；不能确认 DMA 停止时停止发送 |

不会用新原点自动掩盖定位中断。编码器恢复健康后，可在任务上下文调用 `odometry_app_reset()` 建立新的 `(0,0,0)`；这不是恢复旧轨迹。若 IMU 已丢失姿态连续性，需重新启动并静止标定。

`odometry_app_get_output()` 统一检查运行状态、连续性与新鲜度，串口复用这一结果。正常报文格式固定：

```text
$ODOM,seq,t_us,x_mm,y_mm,yaw_deg,valid,continuous,state,error*HH\r\n
```

单位为毫米、度、微秒；TIM2 时间戳约 71.6 分钟回绕，序号为 uint32。校验是 `$` 与 `*` 之间所有字节逐个 XOR。接收端按 `$` 同步、按回车换行取帧，校验通过且 `valid=1` 时才使用位姿，并检测断流。

| 最后四个字段 | 含义 |
| --- | --- |
| `0,0,0,0` | 尚未建立原点 |
| `1,1,1,0` | 正常定位 |
| `0,1,0,5` | 有界同步恢复等待；坐标暂时不可用 |
| `0,0,2,4` | 输入连续性丢失 |
| `0,0,2,5` | 同步无法在现有边界内恢复 |

同步拒绝可能在同一次任务循环内恢复，因此不一定能在 100 Hz 串口中看到等待帧；通过 `g_odometry.skipped_sync_samples/sync_recoveries` 统计观察。两者是诊断计数，不参与几何计算；重建原点时清零。`sync_failure` 保存最近一次拒绝原因，恢复后无需为 0；当前状态以 `state/error/valid` 为准。

## 烧录与装车验收

构建命令：

```powershell
cmake --preset Debug
cmake --build --preset Debug --parallel 4
```

固件为 `build/Debug/LED.elf`。串口只查看输出时，PD5 接 3.3 V USB-TTL 的 RX，GND 共地即可，PD6 可不接。开机保持静止，通常约 3 秒完成初始标定。

验收先观察串口恢复 `1,1,1,0`，序号和采样时间戳持续更新。关键变量只需先看 `g_imu.valid`、`g_encoder.valid`、`g_encoder2.valid`、`g_odometry.state/error` 和 `g_telemetry.tx.errors/timeouts`；定位异常时再展开详细原因。

| 步骤 | 记录与确认内容 | 当前状态 |
| --- | --- | --- |
| 静止连续运行至少 30 分钟 | 记录 X/Y/Yaw、错误计数、两路最大采样间隔、同步拒绝/恢复次数；坐标时间持续更新 | 待本次固件实测 |
| 向前、向左各移动已知距离 | 先确认 +X/+Y 符号；用实测距离标定有效轮径 | 待装车 |
| 顺逆时针原地旋转 | 确认逆时针航向为正，观察中心 X/Y 残差，复核滚动方向和交点偏置 | 待装车 |
| 矩形路线返回起点 | 记录闭环位置和航向误差，分别评估轮径、安装角、打滑及漂移 | 待装车 |
| 接收端联调 | 确认校验、时间戳回绕、断流及 invalid 帧处理 | 待主控联调 |

轮径 58 mm、0°/90° 和方向符号是当前名义参数，实际方向角与装配偏置仍需测量。0.3 °/s 死区会抑制低速真实转动，温漂模型也未标定；这些是用户已决定延后到安装/转台阶段的事项，软件收尾不等于定位精度已验收。

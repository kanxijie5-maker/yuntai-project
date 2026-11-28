# Driver_Bench · GL40 单轴低速台架

适用于 R2.4 单轴驱动板（GD32F303CCT6、DRV8316CR）、GL40 KV70（14 极对）、AS5048A 和 12 V 限流电源。

## 编译与烧录

用 Keil µVision 5 打开 `Driver_Bench.uvprojx`，使用 Arm Compiler 5.06 update 6 和 GigaDevice.GD32F30x_DFP 2.6.0。按 F7 编译，输出为 `build/Keil/Driver_Bench/Driver_Bench.hex`。J-Link 接 J5 的 SWD 接口后按 F8 下载，也可直接使用随工程提供的 [Driver_Bench.hex](Driver_Bench.hex)。

## 运行 1 rpm

1. 固定电机定子，给转轴留出旋转空间。J1 接电源，J6 接 3.3 V USB 转串口。J6-2 接 GND、J6-3 接转接器 RXD、J6-4 接转接器 TXD。J6-1 留空。
2. 串口设为 921600、8N1、无流控、UTF-8 文本。命令末尾加换行。
3. 单发 `motor`，等 `MOTOR PASS` 后发 `rpm1`。电机持续正转。发 `rpm-1` 反转，发 `stop` 停机。
4. 发 `status` 查看运行状态。`COG TABLE=D_SMOOTH READY=1 ON=1` 表示编码器方向匹配、齿槽补偿已就绪。更换电机或改变编码器、磁铁的安装角后，在重新标定前，每次上电先在空闲状态发 `cogoff`。

## 本次低速优化

PWM 为 20 kHz，电流环为 10 kHz，速度环为 1 kHz。速度反馈采用 8 ms 编码器位移差分和一阶低通。电流环采用两点平均，并在调制时加入等效 0.50 µs 的死区补偿。固件用原始机械角查 512 点 D 平滑表，向速度 PI 输出叠加转矩电流前馈。[齿槽表](gl40_cogging_table.h)

2026-11-27 在这台裸电机以 ±1 rpm 测试。关表组正、反转的速度误差 RMS 分别为 0.259、0.244 rpm。两次开启 D 平滑表后，正转均为 0.224 rpm，反转分别为 0.215、0.196 rpm。这里记录的是当日台架实测。整理后的独立工程另行通过编译和软件回归，生成的 HEX 与整理前的台架目标逐字节一致。[实测波形与低转速调试复盘](GL40_低转速抖动调试复盘_2026-11-27.pdf)

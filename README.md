# 平衡车 (STM32F103C8T6 Two-Wheel Balancing Car)

基于 STM32F103C8T6 + MPU6050 + TB6612 的两轮自平衡小车，三环 PID 控制。

## 控制结构

基于江科大平衡车教程 06 的三环 PID 逻辑，参数按实车调优后固化：

- **角度环**（10 ms）：Kp=3.30, Ki=0.15, Kd=3.00 —— 保持直立
- **速度环**（50 ms）：Kp=0.80, Ki=0.065 —— 编码器差速，输出目标倾角串联到角度环
- **转向环**（50 ms）：Kp=3.00, Ki=4.00 —— 编码器差速 PI，防止自转
- **死区补偿**：216 counts（3% 占空比）

姿态解算：互补滤波（α=0.01，10 ms），MPU6050 关闭 DLPF 换取低延迟。
上电 CAL 扶正标定自动把中心角清零。

**固定参数**：角度微调为 0，输出方向、50° 保护、死区、电机极性、编码器方向、
轮动力补偿全部写死在固件常量里，不再通过蓝牙下发。调试期用的 PING/OFFSET/
SPD/YAWSP/TARGET/RSTI/PERF/POL/TRIM/SIGN/PROT/ENC/ENCZ/ENCS 已从固件和上位机
一并移除。链路上保留：`SET/SETV/SETT`（三环增益）、`GET/GETV/GETT`（读取）、
`SAVE`（存 Flash）、`CAL`、`BAL,0/1`、`MOV`（遥控行驶）、`BST`（状态）、`T`（遥测）。

## 运动控制（前进 / 后退 / 左右转）

在「原地直立」的基础上，速度环目标和转向环目标重新开放给上位机：

```
MOV,<fwd>,<turn>      百分比 -100..100，fwd + = 前进，turn + = 右转
```

- `fwd` 写进**速度环目标**（轮速）。速度环是串级结构，输出是「目标倾角」，
  所以车会先主动前倾加速，到达目标速度后自己回正保持匀速。
- `turn` 写进**转向环目标**（左右轮速差）。正差速 = 左轮快 = 向右转；
  前后为 0 时就是原地打转。
- 两个目标都过**斜坡限速**（`APP_BAL_TARGET_SLEW`，满量程约 0.5 s），
  一次按键不会把目标倾角直接顶到 20° 限幅。
- 100% 对应速度 5 rev/s（约 1 m/s）+ 差速 2 rev/s，见 `APP_BAL_SPEED_PER_PCT`
  / `APP_BAL_DIF_PER_PCT`，觉得慢/快改这两个常量，或直接用上位机「限速」滑块。

**失联保护（重要）**：`MOV` 是 *deadman* 命令。上位机每 100 ms 重发一次
（哪怕值没变，只是心跳）；固件 600 ms 收不到就把目标斜坡降到 0 并自动停车。
所以蓝牙掉线、笔记本合盖、关掉上位机，车都会自己停下来，不会带着上一个方向
一路跑掉。用串口助手手动调试时，也需要每 600 ms 至少发一次 `MOV`。

**方向反了**：不用拆电机线，改 `Core/Inc/app_balance.h` 里的
`APP_BAL_MOVE_FWD_SIGN` / `APP_BAL_MOVE_TURN_SIGN` 为 `-1.0f` 重新编译即可。

**上位机操作**：按住 `W/S` 前进后退、`A/D` 左右转，可同时按走出弧线；松手立刻停。
也可以按住界面上的方向按钮（鼠标），或用「停」按钮。焦点在输入框里时按键只
影响输入框，不会误开车。每次「开始平衡」都会清空方向，必须重新按键才动。

## 目录结构

```
Core/              用户源码（app_balance.c 三环控制、app_pid_tune.c 通信协议等）
Drivers/           STM32 HAL 库 + CMSIS
Middlewares/       FreeRTOS
MDK-ARM/           Keil 工程文件 + 可烧录 hex（MDK-ARM/Balance_Car/Balance_Car.hex）
Tools/PID_Tuner/   电脑上位机 PID 调试助手（Python + tkinter）
Tools/Mobile_App/  手机 App 上位机（uni-app + 经典蓝牙 SPP，HBuilderX 打 APK）
Balance_Car.ioc    STM32CubeMX 工程配置
```

## 手机 App（安卓）

`Tools/Mobile_App/` 是一个 uni-app 工程，功能对齐电脑上位机：驾驶（前后/左右双杆摇杆 + 限速）、
三环 7 项增益调参、实时波形、参数存档、通信日志。**固件不用改**，发的是同一套协议。

关键点：HC-05 是经典蓝牙 SPP，而 uni-app 的蓝牙 API 只支持 BLE，所以这里用
Native.js 直接调安卓原生 `BluetoothSocket`（`common/spp.js`）。HBuilderX 云打包
安卓 APK 免费（用「云端证书」，2026 年起公共测试证书已停用），超出免费额度 2 元/次。

详细步骤见 `Tools/Mobile_App/README.md`：装 HBuilderX → 补 AppID → 配对 HC-05 →
真机基座调通 → 云打包出 APK。调试阶段用「运行到手机基座」不消耗打包次数。

## 上位机

需要 Python 3 + tkinter。运行：

```bash
python Tools/PID_Tuner/pid_tuner.py
```

功能：实时波形显示、三环 7 项增益的滑块/键盘上下键即时调参、扶正标定、
开始/急停平衡、方向键遥控行驶、CSV 数据导出、参数存档。界面只保留调参、
平衡与行驶必需项，测试/固定类按钮（死区下发、测周期、极性、编码器方向、
动力补偿等）已移除。

**参数存档**：调出满意的参数后，在左侧「参数存档」面板输入名字点「保存当前」，
会把 7 项增益和死区补偿值（仅作记录，固件里已固定）存到脚本同目录的
`presets.json`（随仓库备份）。可一键「载入并下发」或「仅填入不发送」，
也能「导出对比CSV」横向比较各版本参数。

**掉电保存**：点「保存到小车（断电不丢）」会把当前 7 项增益通过 `SAVE` 命令
写入小车片内 Flash 最后一页（`bsp_flash_params.c`，magic+CRC 校验），每次上电
自动恢复。运行中保存会被固件拒绝（页擦除会卡住 CPU 约 30 ms），先急停再存。
擦写寿命约 1 万次，供偶尔保存足够；想回到出厂参考值，点「恢复参考值」后再点
一次「保存到小车」覆盖即可。

保存到底成没成功，看接收区最后一行：

| 回执 | 含义 | 处理 |
| --- | --- | --- |
| `LOG,Params saved to flash - survives power off` | 写入成功 | 断电重上电，会看到 `restored from flash` |
| `LOG,ERR SAVE rejected - BAL,0 first` | 还在 RUN/CAL | 先点急停，等状态变 IDLE 再存 |
| `LOG,ERR SAVE flash write failed` | 页被写保护/读保护 | 检查选项字节里的保护位 |
| `LOG,ERR unknown cmd` | 固件里没 SAVE | 烧的是旧 hex，重新编译烧录 |

上位机标签会自动变成「✅ 已存入小车 Flash」，3 秒无回执则提示检查串口。

## 烧录

用 Keil 打开 `MDK-ARM/Balance_Car.uvprojx` 编译，或直接烧录 `MDK-ARM/Balance_Car/Balance_Car.hex`。
烧录后按顺序：架空静止 0.5s（标陀螺零偏）→ 扶正标定 CAL → 点"恢复参考值" → 开始平衡 BAL,1
→ 按住 W 试试前进（第一次先用手虚扶着，随时急停）。

## 硬件

- MCU: STM32F103C8T6
- IMU: MPU6050 (I2C 软件模拟)
- 电机驱动: TB6612FNG
- 编码器: 减速电机自带霍尔编码器
- 蓝牙: HC-05（串口调参）

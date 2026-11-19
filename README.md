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

## 目录结构

```
Core/              用户源码（app_balance.c 三环控制、app_pid_tune.c 通信协议等）
Drivers/           STM32 HAL 库 + CMSIS
Middlewares/       FreeRTOS
MDK-ARM/           Keil 工程文件 + 可烧录 hex（MDK-ARM/Balance_Car/Balance_Car.hex）
Tools/PID_Tuner/   上位机 PID 调试助手（Python + tkinter）
Balance_Car.ioc    STM32CubeMX 工程配置
```

## 上位机

需要 Python 3 + tkinter。运行：

```bash
python Tools/PID_Tuner/pid_tuner.py
```

功能：实时波形显示、滑块/键盘上下键调参即时下发、CSV 数据导出、死区补偿下发等。

**参数存档**：调出满意的参数后，在左侧「参数存档」面板输入名字点「保存当前」，
会把 7 项增益和死区补偿存到脚本同目录的 `presets.json`（随仓库备份）。
可一键「载入并下发」或「仅填入不发送」，也能「导出对比CSV」横向比较各版本参数。

## 烧录

用 Keil 打开 `MDK-ARM/Balance_Car.uvprojx` 编译，或直接烧录 `MDK-ARM/Balance_Car/Balance_Car.hex`。
烧录后按顺序：架空静止 0.5s（标陀螺零偏）→ 扶正标定 CAL → 点"恢复参考值" → 开始平衡 BAL,1。

## 硬件

- MCU: STM32F103C8T6
- IMU: MPU6050 (I2C 软件模拟)
- 电机驱动: TB6612FNG
- 编码器: 减速电机自带霍尔编码器
- 蓝牙: HC-05（串口调参）

# bldc_foc

面向**矮化密植苹果园采摘机器人**底盘/关节的 BLDC 驱动工程。
当前版本用 **六步换相（霍尔闭环）** 跑通功率级，后续在同一套 BSP 上叠加 FOC。

| 项 | 配置 |
|---|---|
| 主控 | 正点原子 探索者 **STM32F407ZGT6**（Cortex-M4F @168MHz） |
| 功率板 | **POWER_DRV8353HR_V1**（TI DRV8353HR，12–60V / 连续 26A / 峰值 28A） |
| 电机 | 24V / 20A BLDC，带 3 路霍尔（HA/HB/HC） |
| 工具链 | Keil MDK5 + **AC5**（ARMCC V5.06u5）+ STM32F4xx DFP |

> 编译状态：`0 error 0 warning`，Code 16012 B / RO 740 B / RW 116 B / ZI 1412 B。

---

## 目录结构

```
.
├── firmware_drv8353/          # Keil 工程 + 全部源码
│   ├── Core/
│   │   ├── Inc/ bsp_pinmap.h  bsp_drv8353.h  bsp_hall.h  bsp_adc.h  bsp_time.h  bsp_uart.h
│   │   └── Src/ main.c  bsp_*.c  stm32f4xx_it.c
│   ├── Drivers/               # STM32Cube HAL V1.28.3（含 ADC / TIM / UART）
│   ├── MDK-ARM/DRV8353.uvprojx  ← 打开这个
│   └── README.md              ← 接线表 + 调试步骤（先读这个）
├── .gitignore
└── .gitattributes
```

## 快速开始

```bash
git clone https://github.com/yan999777888/bldc_foc.git
```

1. 用 Keil 打开 `firmware_drv8353/MDK-ARM/DRV8353.uvprojx`，直接 Build（F7）。
2. **接线前先读** [`firmware_drv8353/README.md`](firmware_drv8353/README.md) 第二节的 P2 引脚表。
3. 有三个前提做不到就会烧 IO，务必确认：
   - **拔掉 P6 跳线帽**（否则 PA9/PA10 上 TIM1_CH2/CH3 与板载 CH340 对打）
   - **拔掉 P12 跳线帽**（否则 PA5 被 TPAD 占用）
   - **不插** OLED / CAMERA / TFTLCD / SD 卡 / NRF24L01
4. 调试串口走 **USART3（PB10/PB11，115200-8-N-1）**，P10 跳线打到 ATK-MODULE 侧。

## 引脚分配（摘要）

| 功能 | 引脚 | 说明 |
|---|---|---|
| TIM1_CH1 / CH1N | PA8 / PB13 | A 相上下桥 |
| TIM1_CH2 / CH2N | PA9 / PB14 | B 相上下桥 |
| TIM1_CH3 / CH3N | PA10 / PB15 | C 相上下桥 |
| DRV_EN | PG6 | 板上下拉，默认关断 |
| FAULT | PG7 | 低有效，板载 4.7k 上拉 |
| HALL HA/HB/HC | PB6 / PB7 / **PC8** | EXTI6/7/8（不能都用 PBx，EXTI 按线号冲突） |
| IOUTA/IOUTB/IOUTC | PA5 / PA4 / PB0 | ADC1 三相电流 |
| VBUS / CBUS / NTC | PC3 / PB1 / PC1 | 母线电压 / 母线电流 / 温度 |

## 工程要点

- **20kHz 三相互补 PWM**，死区由 MCU 插入（`BDTR.DTG=84` → 500ns）。6×PWM 模式下 DRV8353 不代插死区。
- **悬空相处理**：清 `CCxE/CCxNE` 之外必须置 `OSSR=1`，否则引脚变高阻、DRV 输入状态不定。
- **静止起动**：霍尔闭环靠边沿换相，静止时没有边沿 → 必须先按当前扇区通电，再交给中断。
- 六步换相表：

  | 霍尔 | 导通 |
  |---|---|
  | 100 | C+ B− |
  | 110 | C+ A− |
  | 010 | B+ A− |
  | 011 | B+ C− |
  | 001 | A+ C− |
  | 101 | A+ B− |

- **ADC**：6 路 DMA 循环 + 上电零点标定；量程见 `bsp_adc.h` 注释。
- **保护**：过流 / 过压 / 欠压 / 过温 / FAULT 五重软保护，1ms 轮询。

## 已知限制

1. 无电流环、无速度闭环，只有软起动 + 固定占空比。
2. 相电流分辨率 **81 mA/LSB**（GAIN=5V/V、2mΩ → 量程 ±165A）。做电流环前把功率板 GAIN 跳到 20V/V（R43、R47 都不焊）→ ±41A / 20mA per LSB。
3. ADC 采样率约 7kHz（480 周期采样，照顾 124k 分压的高阻源）；做电流环需缩短采样时间。
4. `printf` 阻塞发送，115200 下一行约 10ms，只能 1Hz，严禁进控制环。
5. 保护是"软"的，短路级故障仍依赖功率板自身的 VDS 保护（阈值约 300A 起，只防短路）。

## 路线图

- [x] 六步换相 + 霍尔闭环 + 软起动
- [x] 六路模拟量采集与标定
- [x] 五重软保护
- [ ] 速度 PI（1kHz 环，用 `Hall_GetSpeedRpm()`）
- [ ] 母线电流限幅（外环限幅 + `DRV_SetDuty()` 钳位）
- [ ] GAIN 改档 + 双电阻采样 → FOC 电流环

## 编码注意

`Core/` 下的 `.c/.h` 含中文注释，**按 GB2312 保存**（AC5 对 UTF-8 中文会报 `invalid multibyte char`，且"启""转"等字尾字节为 `0x5C`，会吞掉后面的引号报 `missing closing quote`）。
GitHub 网页直接查看这些文件时中文注释会显示为乱码，属正常现象，克隆到本地用 Keil / GB2312 编辑器打开即正常。

## License

见 [LICENSE](LICENSE)。

# 小喵掌机 ESP-IDF 固件与硬件资料

这是给学而思小喵掌机移植的 ESP-IDF / LVGL 固件工程，同时整理了屏幕、按键、传感器、GD32 协处理器和底层协议等硬件资料。

## 固件下载与刷入

已经编译好的 merged bin 会放在本项目的 [Releases](https://github.com/ZyoungInc/xueersi-idf/releases/latest) 页面。普通用户可以直接下载 release 里的 `xiaomiao-merged.bin` 并从 `0x0` 地址刷入，不需要自己搭建 ESP-IDF 编译环境。

示例命令：

```bash
esptool.py --chip esp32 -b 460800 write_flash 0x0 xiaomiao-merged.bin
```

刷入前请确认目标硬件是 ESP32-WROVER-B 版本的小喵掌机，并确认串口连接正常。

## 当前状态

- ESP32 侧固件已经移植到 ESP-IDF，使用 LVGL 9.x 驱动 ST7735/ST7785 类 SPI 屏幕，并提供硬件状态分页 UI。
- 光照、热敏、蜂鸣器、按键、MicroSD、I2C 设备探测等功能已经接入 ESP32 侧固件。
- GD32 固件仍在开发中，目前公开的 GD32 工程主要用于 USB 转串口功能。
- GD32 与 ESP32 间的 LED、电机等控制协议仍在整理，正在考虑是否兼容原有协议。欢迎大家在 Issues 里提出建议。

## 原理图与鸣谢

原理图文件已整理为 [`xueersi-xiaomiao-schematic.pdf`](xueersi-xiaomiao-schematic.pdf)。

感谢 ID「我为电波狂」对硬件进行测量并制作原理图，这部分资料对后续移植和维护非常关键。

## 参与项目

如果这个项目对你有帮助，欢迎 Star。遇到问题、发现硬件差异、或者有协议兼容建议，可以提交 Issue。也欢迎提交 PR，我会审核后合并。

## 使用与署名要求

二次开发、转载、分发、商用或以任何形式使用本项目内容时，必须保留并明确引用原作者与本项目来源，以尊重劳动成果。违反者将依法追究责任；本人保留所有权利。

## 1. 总体架构

```text
PC USB
  │
  │ USB CDC / 下载串口
  ▼
GD32F350G8
  ├─ USB 转 ESP32 UART0
  ├─ ESP32 自动下载 / 自动复位控制
  ├─ I2C 从机地址 0x40
  ├─ 控制双路电机驱动 HR8833 / DRV8833
  └─ 控制板载 LED1 / LED2

ESP32-WROVER-B
  ├─ SPI TFT 显示屏
  ├─ MicroSD 卡
  ├─ 6 个按键
  ├─ 蜂鸣器 PWM
  ├─ 光照 ADC
  ├─ 热敏 ADC
  ├─ I2C 主机
  │   ├─ GD32F350G8：0x40
  │   └─ MPU6050：0x68
  └─ 预留扩展 IO
```

核心关系：

```text
ESP32 = 主控 / UI / Python 运行环境 / 屏幕 / SD / 按键 / 传感器
GD32  = USB 串口桥 / ESP32 自动烧录控制 / 电机与 LED 控制器
0x40  = GD32 的 I2C 从机地址
```

***

## 2. ESP32 引脚分配

### 2.1 TFT 显示屏

| 功能             | ESP32 引脚   |
| -------------- | ---------- |
| SPI SCK        | GPIO18     |
| SPI MOSI       | GPIO23     |
| SPI MISO       | GPIO19     |
| TFT DC         | GPIO4      |
| TFT CS         | GPIO5      |
| TFT RES / 相关复用 | GPIO19     |

显示对象信息：

```text
显示分辨率：160 × 128
SPI：SPI2
SPI 频率：40 MHz
SCK：GPIO18
MOSI：GPIO23
MISO：GPIO19
DC：GPIO4
```

屏幕底层通过 `FrameBuffer` 和 `SCREEN` 对象刷新。

***

### 2.2 MicroSD 卡

| 功能         | ESP32 引脚   |
| ---------- | ---------- |
| SPI SCK    | GPIO18     |
| SPI MOSI   | GPIO23     |
| SPI MISO   | GPIO19     |
| SD CS      | GPIO22     |

TFT 与 MicroSD 共用 SPI2，通过不同 CS 分时使用。

***

### 2.3 按键

| 按键         | ESP32 引脚   |
| ---------- | ---------- |
| 上          | GPIO2      |
| 下          | GPIO13     |
| 左          | GPIO27     |
| 右          | GPIO35     |
| A          | GPIO34     |
| B          | GPIO12     |

注意：

```text
GPIO34、GPIO35 是输入专用脚。
GPIO12 是启动相关敏感脚。
```

***

### 2.4 ADC 传感器

| 功能         | ESP32 引脚   |
| ---------- | ---------- |
| 光照传感器      | GPIO36     |
| 热敏电阻       | GPIO39     |

已确认：

```text
sensor.getLight() = ADC(GPIO36).read()
sensor.getTemp()  = ADC(GPIO39) 后换算
```

***

### 2.5 蜂鸣器

| 功能         | ESP32 引脚   |
| ---------- | ---------- |
| 无源蜂鸣器      | GPIO14     |

底层对象：

```text
PWM(14, freq=440, duty=0)
```

也就是：

```text
GPIO14 → PWM → 无源蜂鸣器
```

***

### 2.6 I2C 总线

| 功能         | ESP32 引脚   |
| ---------- | ---------- |
| I2C SCL    | GPIO15     |
| I2C SDA    | GPIO21     |

I2C 设备：

| 地址         | 设备         |
| ---------- | ---------- |
| 0x40       | GD32F350G8 |
| 0x68       | MPU6050    |

当前已确认：

```text
GD32：0x40
MPU6050：0x68，未安装时不会出现在 scan 结果中
```

***

### 2.7 UART0

| 功能         | ESP32 引脚   |
| ---------- | ---------- |
| UART0 TX   | GPIO1      |
| UART0 RX   | GPIO3      |

该 UART0 通过 GD32 转 USB 与电脑通信，用于 Python 终端、程序上传、ESP32 自动下载。

***

## 3. GD32F350G8 连接关系

GD32F350G8 在板上承担以下功能：

```text
1. USB CDC 串口桥
2. ESP32 自动复位 / 自动下载控制
3. I2C 从机 0x40
4. LED1 / LED2 控制
5. 双路电机控制
6. HR8833 / DRV8833 控制信号输出
```

已知相关连接：

| 功能          | 连接对象                     |
| ----------- | ------------------------ |
| USB D+ / D- | USB 接口                   |
| UART 桥      | ESP32 GPIO1 / GPIO3      |
| 自动下载控制      | ESP32 IO0 / 复位相关线路       |
| I2C         | ESP32 GPIO15 / GPIO21    |
| 电机 PWM      | HR8833 / DRV8833         |
| LED 控制      | LED1 / LED2              |
| SWD         | TMS / TCK / RST / GND 焊盘 |

***

## 4. I2C 地址与设备

### 4.1 I2C 总线

```text
I2C 控制器：ESP32 I2C(0)
SCL：GPIO15
SDA：GPIO21
频率：100 kHz
```

### 4.2 地址表

| I2C 地址     | 设备         | 说明         |
| ---------- | ---------- | ---------- |
| 0x40       | GD32F350G8 | LED、电机控制   |
| 0x68       | MPU6050    | 加速度计 / 陀螺仪 |

***

## 5. GD32 0x40 协议

## 5.1 LED 协议

GD32 的 LED 控制使用 I2C memory write 形式。

| 功能         | I2C 地址     | 寄存器        | 数据         |
| ---------- | ---------- | ---------- | ---------- |
| LED1 关闭    | 0x40       | 0xA0       | 0          |
| LED1 打开    | 0x40       | 0xA0       | 1          |
| LED2 关闭    | 0x40       | 0xA1       | 0          |
| LED2 打开    | 0x40       | 0xA1       | 1          |

LED 对象内部状态：

```text
LED1:
  reg = 0xA0

LED2:
  reg = 0xA1
```

***

## 5.2 电机协议概览

电机控制通过 I2C 向 0x40 写入一组 PWM 寄存器格式数据。

基本格式：

```text
I2C 地址：0x40

数据格式：
[
  起始寄存器,
  通道A_ON_L,
  通道A_ON_H,
  通道A_OFF_L,
  通道A_OFF_H,
  通道B_ON_L,
  通道B_ON_H,
  通道B_OFF_L,
  通道B_OFF_H
]
```

每个电机占两个 PWM 通道：

```text
一个通道控制一个方向输入。
另一个通道控制反方向输入。
```

方向逻辑：

```text
方向 1：
  IN_A = PWM
  IN_B = 0

方向 0：
  IN_A = 0
  IN_B = PWM
```

***

## 5.3 电机编号与寄存器

| 电机编号       | 起始寄存器      | 占用通道         |
| ---------- | ---------- | ------------ |
| Motor 2    | 0x06       | PWM 通道 0 / 1 |
| Motor 1    | 0x0E       | PWM 通道 2 / 3 |

***

## 5.4 速度映射

速度参数范围：

```text
speed = 0 ~ 255
```

转换关系：

```text
PWM_12bit = speed × 16
PWM_12bit = speed << 4
```

示例：

| speed      | PWM 十进制    | PWM 十六进制   | 低字节        | 高字节        |
| ---------- | ---------- | ---------- | ---------- | ---------- |
| 0          | 0          | 0x0000     | 0x00       | 0x00       |
| 1          | 16         | 0x0010     | 0x10       | 0x00       |
| 10         | 160        | 0x00A0     | 0xA0       | 0x00       |
| 50         | 800        | 0x0320     | 0x20       | 0x03       |
| 100        | 1600       | 0x0640     | 0x40       | 0x06       |
| 128        | 2048       | 0x0800     | 0x00       | 0x08       |
| 200        | 3200       | 0x0C80     | 0x80       | 0x0C       |
| 255        | 4080       | 0x0FF0     | 0xF0       | 0x0F       |

***

## 5.5 电机 1 数据格式

### Motor 1，方向 1

```text
[
  0x0E,
  0x00, 0x00, PWM_L, PWM_H,
  0x00, 0x00, 0x00, 0x00
]
```

### Motor 1，方向 0

```text
[
  0x0E,
  0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, PWM_L, PWM_H
]
```

***

## 5.6 电机 2 数据格式

### Motor 2，方向 1

```text
[
  0x06,
  0x00, 0x00, PWM_L, PWM_H,
  0x00, 0x00, 0x00, 0x00
]
```

### Motor 2，方向 0

```text
[
  0x06,
  0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, PWM_L, PWM_H
]
```

***

## 5.7 全部电机停止

全停命令：

```text
I2C 地址：0x40
数据：[0x00, 0x00, 0x00, 0x00, 0x00]
```

***

## 6. MPU6050

### 6.1 总线连接

| 功能         | ESP32 引脚   |
| ---------- | ---------- |
| SCL        | GPIO15     |
| SDA        | GPIO21     |

### 6.2 地址

```text
MPU6050 I2C 地址：0x68
```

### 6.3 传感器对象内部字段

```text
addr = 104 = 0x68
imuReady = True / False
imu = 14 字节缓存
```

### 6.4 数据类型

MPU6050 提供：

```text
accX
accY
accZ
gyroX
gyroY
gyroZ
pitch
roll
gesture
```

***

## 7. SugarASR 扩展占用

`sugar_asr.py` 使用：

```text
UART1
TX = GPIO21
RX = GPIO15
波特率 = 115200
```

这与板载 I2C 引脚重合：

```text
GPIO21 = I2C SDA
GPIO15 = I2C SCL
```

因此：

```text
使用 SugarASR 时，GPIO15 / GPIO21 会作为 UART1 使用。
使用 LED、电机、MPU6050 时，GPIO15 / GPIO21 会作为 I2C 使用。
```

***

## 8. 板载扩展 IO

板上预留扩展 IO：

| ESP32 GPIO | 类型               | 说明           |
| ---------- | ---------------- | ------------ |
| GPIO33     | GPIO / ADC       | 可作输入、输出、ADC  |
| GPIO32     | GPIO / ADC       | 可作输入、输出、ADC  |
| GPIO26     | GPIO / DAC / PWM | 可作输出、PWM、DAC |
| GPIO25     | GPIO / DAC / PWM | 可作输出、PWM、DAC |

推荐作为普通扩展口使用的 ESP32 引脚：

```text
GPIO25
GPIO26
GPIO32
GPIO33
```

***

## 9. ESP32 引脚占用总表

| GPIO       | 当前功能                  | 备注               |
| ---------- | --------------------- | ---------------- |
| GPIO1      | UART0 TX              | 经 GD32 转 USB 串口  |
| GPIO2      | 上键                    | 输入               |
| GPIO3      | UART0 RX              | 经 GD32 转 USB 串口  |
| GPIO4      | TFT DC                | 显示               |
| GPIO5      | TFT CS                | 显示               |
| GPIO12     | B 键                   | 启动相关敏感脚          |
| GPIO13     | 下键                    | 输入               |
| GPIO14     | 蜂鸣器                   | PWM              |
| GPIO15     | I2C SCL / SugarASR RX | 与 GPIO21 成组复用    |
| GPIO18     | SPI SCK               | TFT / SD 共用      |
| GPIO19     | SPI MISO / TFT 相关     | TFT / SD 相关      |
| GPIO21     | I2C SDA / SugarASR TX | 与 GPIO15 成组复用    |
| GPIO22     | SD CS                 | MicroSD          |
| GPIO23     | SPI MOSI              | TFT / SD 共用      |
| GPIO25     | 预留扩展                  | GPIO / DAC / PWM |
| GPIO26     | 预留扩展                  | GPIO / DAC / PWM |
| GPIO27     | 左键                    | 输入               |
| GPIO32     | 预留扩展                  | GPIO / ADC       |
| GPIO33     | 预留扩展                  | GPIO / ADC       |
| GPIO34     | A 键                   | 输入专用             |
| GPIO35     | 右键                    | 输入专用             |
| GPIO36     | 光照 ADC                | 输入专用             |
| GPIO39     | 热敏 ADC                | 输入专用             |

***

## 10. 开发用对象映射

| Python 对象 / 模块       | 底层硬件               |
| -------------------- | ------------------ |
| `screen`             | FrameBuffer + TFT  |
| `display`            | 160 × 128 TFT 显示封装 |
| `tft`                | 底层 SCREEN 对象       |
| `fb` / `fbuf`        | FrameBuffer        |
| `vspi`               | SPI2，40 MHz        |
| `i2c`                | I2C0，SCL=15，SDA=21 |
| `led1`               | GD32 0x40，寄存器 0xA0 |
| `led2`               | GD32 0x40，寄存器 0xA1 |
| `buzzer`             | GPIO14 PWM         |
| `sensor.adcLight`    | GPIO36 ADC         |
| `sensor.adcTemp`     | GPIO39 ADC         |
| `sensor.btns`        | 6 个 ESP32 GPIO 按键  |
| `motor.Motor`        | GD32 0x40 电机协议     |
| `sugar_asr.SugarASR` | UART1，TX=21，RX=15  |

***

## 11. 开发时可直接使用的底层信息

### 11.1 I2C

```text
I2C0:
  SCL = GPIO15
  SDA = GPIO21
  freq = 100000

设备：
  0x40 = GD32
  0x68 = MPU6050
```

### 11.2 LED

```text
LED1:
  addr = 0x40
  reg  = 0xA0
  value 0 = off
  value 1 = on

LED2:
  addr = 0x40
  reg  = 0xA1
  value 0 = off
  value 1 = on
```

### 11.3 电机

```text
Motor 1:
  起始寄存器 = 0x0E

Motor 2:
  起始寄存器 = 0x06

speed:
  0 ~ 255
  PWM = speed << 4

direction:
  1 = 第一方向通道 PWM，第二方向通道 0
  0 = 第一方向通道 0，第二方向通道 PWM
```

### 11.4 蜂鸣器

```text
GPIO14
PWM 输出
无源蜂鸣器
```

### 11.5 光照 / 温度

```text
光照：
  GPIO36
  ADC

热敏：
  GPIO39
  ADC
```

### 11.6 按键

```text
up    = GPIO2
down  = GPIO13
left  = GPIO27
right = GPIO35
a     = GPIO34
b     = GPIO12
```

### 11.7 显示

```text
分辨率：160 × 128
SPI：SPI2
SCK：GPIO18
MOSI：GPIO23
MISO：GPIO19
DC：GPIO4
CS：GPIO5
```

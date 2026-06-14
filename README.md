# CraftyHandy ESP32 SR6 Tool

> Website: [craftyhandy.com](https://craftyhandy.com/) | Email: openosr@gmail.com

---

## 🇨🇳 中文

### 简介
基于浏览器的 ESP32 SR6 调平工具，支持固件烧录、舵机校准、温度保护、速度微调、WiFi 控制、WiFi 配网、WebSocket 脚本播放等功能。

### 固件版本

| 固件 | 文件名 | 说明 |
|------|--------|------|
| **带温控版 (10K)** | `SR6_OSR2_CraftyHandy_ESP32-NTC-10K.bin` | 配备 NTC 温控 PCB (10K 3435)，超温自动暂停 + LED 报警 |
| **带温控版 (5K)** | `SR6_OSR2_CraftyHandy_ESP32-NTC-5K.bin` | 配备 NTC 温控 PCB (5K 3470)，超温自动暂停 + LED 报警 |
| **无温控版** | `SR6_OSR2_CraftyHandy_ESP32-NoNTC.bin` | 无温控硬件的老用户使用，纯运动控制 |

选择对应版本的 `.bin` 文件烧录即可。

### 引脚映射

| 舵机 | GPIO |
|------|------|
| LowerLeft | 15 |
| UpperLeft | 2 |
| LowerRight | 13 |
| UpperRight | 12 |
| LeftPitch | 4 |
| RightPitch | 14 |
| Twist | 27 |
| Valve | 25 |
| TwistFeedback | 26 |
| Vibe0 | 18 |
| Vibe1 | 19 |
| Lube (选配) | 23 |
| NTC (温控版) | 34 |
| Alarm LED (温控版) | 32 |

> **注意**：Valve 在 GPIO25，TwistFeedback 在 GPIO26，与官方 OSR2 工具兼容。

### 固件命令

| 命令 | 功能 | 格式 |
|------|------|------|
| D0 | 查询固件版本 | — |
| D3 / $Z | 调平值读写 | `$Z1500,1500,1500,1500,1500,1500` |
| D4 / $T | 温度设置(仅NTC版) | `$T60.0,3.0` |
| D5 / $S | 速度系数 | `$S1.0,1.0,1.0,1.0,1.0,1.0` |
| D6 / $B | 蓝牙名称 | `$BMyDevice` |
| D7 / $W | WiFi配置 | `$WMySSID,MyPassword` |
| DSTOP | 紧急停止 | — |
| L0-R2 | 6轴控制 | `L05000` / `R07500` |

> 无温控版发送 D4 会返回 `TEMP: N/A (无温控版本)`。

### 使用方式
1. 用 **Chrome/Edge** 浏览器打开 `index.html`
2. 将 ESP32 通过 USB 连接到电脑
3. 点击「烧录」标签 → 连接设备 → 写入固件
4. 点击「校准」标签 → 连接串口 → 开始校准
5. 点击「WiFi」标签 → 连接串口 → 配网/WebSocket连接

### 功能说明

| 功能 | 说明 |
|------|------|
| **烧录固件** | 连接设备 → 选择 .bin → 写入/擦除/重启 |
| **舵机校准** | SR6(6轴) / OSR2(3轴) 模式切换，±10 微调舵机位置 |
| **温度保护** | 设置温度阈值，超过自动暂停运动 **(仅NTC版)** |
| **速度微调** | 6路舵机独立速度调节（0.1~5.0倍）|
| **测试动作** | 滑条控制 + .funscript 脚本播放 |
| **蓝牙名称** | 修改蓝牙名称 |
| **WiFi配网** | 通过串口配置WiFi SSID/密码并保存到EEPROM |
| **WebSocket连接** | 通过WiFi无线发送TCode命令，支持脚本播放 |

---

## 🇬🇧 English

### Introduction
Browser-based ESP32 SR6 calibration tool with firmware flashing, servo calibration, temperature protection, speed tuning, WiFi control, and WebSocket script playback.

### Firmware Versions

| Firmware | File | Description |
|----------|------|-------------|
| **With NTC (10K)** | `SR6_OSR2_CraftyHandy_ESP32-NTC-10K.bin` | For boards with NTC temp sensor (10K 3435), auto-pause on overheat + LED alert |
| **With NTC (5K)** | `SR6_OSR2_CraftyHandy_ESP32-NTC-5K.bin` | For boards with NTC temp sensor (5K 3470), auto-pause on overheat + LED alert |
| **Without NTC** | `SR6_OSR2_CraftyHandy_ESP32-NoNTC.bin` | For legacy users without NTC hardware, pure motion control |

Choose the `.bin` that matches your hardware.

### Pin Mapping

| Servo | GPIO |
|-------|------|
| LowerLeft | 15 |
| UpperLeft | 2 |
| LowerRight | 13 |
| UpperRight | 12 |
| LeftPitch | 4 |
| RightPitch | 14 |
| Twist | 27 |
| Valve | 25 |
| TwistFeedback | 26 |
| Vibe0 | 18 |
| Vibe1 | 19 |
| Lube (optional) | 23 |
| NTC (temp version) | 34 |
| Alarm LED (temp version) | 32 |

> **Note**: Valve is on GPIO25, TwistFeedback on GPIO26, compatible with official OSR2 tool.

### Firmware Commands

| Command | Function | Format |
|---------|----------|--------|
| D0 | Query firmware version | — |
| D3 / $Z | Read/Write calibration | `$Z1500,1500,1500,1500,1500,1500` |
| D4 / $T | Temperature settings (NTC only) | `$T60.0,3.0` |
| D5 / $S | Speed multiplier | `$S1.0,1.0,1.0,1.0,1.0,1.0` |
| D6 / $B | Bluetooth name | `$BMyDevice` |
| D7 / $W | WiFi config | `$WMySSID,MyPassword` |
| DSTOP | Emergency stop | — |
| L0-R2 | 6-axis control | `L05000` / `R07500` |

> Non-NTC version returns `TEMP: N/A (No temp version)` on D4.

### How to Use
1. Open `index.html` with **Chrome/Edge**
2. Connect ESP32 via USB
3. Click "Flash" tab → Connect → Write firmware
4. Click "Calibration" tab → Connect serial → Start calibrating
5. Click "WiFi" tab → Connect serial → Config
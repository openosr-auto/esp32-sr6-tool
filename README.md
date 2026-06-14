# CraftyHandy ESP32 SR6 Tool

> Website: [craftyhandy.com](https://craftyhandy.com/) | Email: openosr@gmail.com
> Web Tool: [ESP32 SR6 Tool](https://openosr-auto.github.io/esp32-sr6-tool/)

---

## 🇨🇳 中文

### 简介
基于浏览器的 ESP32 SR6 调平工具，支持固件烧录、舵机校准、温度保护、速度微调、WiFi 配网、WebSocket 远程控制、蓝牙名称修改、多语言界面等功能。

**特色**：所有功能均通过网页操作，无需安装任何软件（仅需 Chrome/Edge 浏览器）。

### 固件版本

| 固件 | .ino 源码目录 | 说明 |
|------|-------------|------|
| **带温控版 (10K 3435)** | `SR6_OSR2_CraftyHandy_ESP32-NTC-10K/` | 配备 NTC 温控 PCB (10K 3435)，超温自动暂停 + LED 报警 |
| **带温控版 (5K 3470)** | `SR6_OSR2_CraftyHandy_ESP32-NTC-5K/` | 配备 NTC 温控 PCB (5K 3470)，超温自动暂停 + LED 报警 |
| **无温控版** | `SR6_OSR2_CraftyHandy_ESP32-NoNTC/` | 无温控硬件的老用户使用，纯运动控制 |

选择对应版本的 `.ino` 用 Arduino IDE 编译烧录，或使用 `build/` 目录下的 `.bin` 文件通过网页烧录。

### 网页工具 (`index_wifi.html`)

#### 标签页说明

| 标签 | 功能 |
|------|------|
| **烧录** | 连接 ESP32、选择 .bin 固件、擦除 Flash、写入固件、重启设备 |
| **校准** | SR6(6轴) / OSR2(3轴) 模式切换、舵机调平值 ±10 微调、温度保护设置、速度系数调节、TCode 测试动作、funscript 脚本播放 |
| **网络设置** | 串口连接管理、WiFi SSID/密码配置、蓝牙名称修改、WiFi 远程 TCode 控制（WebSocket）、funscript 远程播放 |

#### 使用流程

**首次使用（固件烧录）**
1. 用 **Chrome/Edge** 浏览器打开 `index_wifi.html`
2. 点击「烧录」标签 → 连接设备 → 选择 .bin → 写入固件 → 断开

**舵机校准**
1. 点击「校准」标签 → 连接串口 → 读取/写入调平值、温度、速度参数
2. 用 TCode 滑块或 funscript 进行动作测试

**WiFi 配网 & 远程控制**
1. 点击「网络设置」标签 → 串口连接
2. 在 WiFi 配置区输入 SSID/密码 → 点写入 → **必须完全断电（拔掉USB）再重新上电，ESP32 才会加载新配置并连接 WiFi**
3. ESP32 重启连接路由器后，在下方输入 IP 地址 → 点连接
4. 连接成功后即可通过滑块、预设按钮、funscript 远程驱动设备

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
| Valve | 26 |
| TwistFeedback | 25 |
| Vibe0 | 18 |
| Vibe1 | 19 |
| Lube (选配) | 23 |
| NTC (温控版) | 34 |
| Alarm LED (温控版) | 32 |

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

### 多语言支持

页面右上角语言切换器支持 **中文 / English / 日本語 / 한국어** 四语界面。

### 蓝牙名称持久化

通过 `$B` 命令修改蓝牙名称后，固件会自动保存到 EEPROM，断电重启后仍保持修改后的名称。重启时会从 EEPROM 加载并重启蓝牙。

### WiFi 配置持久化

通过 `$W` 命令保存 WiFi SSID/密码到 EEPROM，断电后不丢失。**必须完全断电（拔掉USB/断开电源）重新上电后**，ESP32 才会从 EEPROM 读取配置并自动连接 WiFi。仅按复位键(RST)不会加载新配置。

### NTC 温度保护（温控版）

- 每秒检测一次 NTC 温度
- 超过设定阈值 → 自动暂停所有运动 + LED 闪烁报警
- 温度下降回阈值以下 → 自动恢复运动
- **传感器异常保护**：NTC 开路/短路时读取到无效温度（超出 -10°C ~ 120°C 范围），自动跳过保护逻辑，不会误触暂停

### WebSocket 远程控制

ESP32 连接 WiFi 后自动启动 WebSocket 服务器（端口 81）。网页通过 WebSocket 发送 TCode 命令，实现低延迟无线控制。支持：

- 手动 TCode 命令输入
- 6 轴实时滑块
- 预设动作按钮（归位、归零、扫描）
- Funscript 远程播放（支持原速/100ms/200ms/500ms 速度调节）

---

## 🇬🇧 English

### Introduction
Browser-based ESP32 SR6 tool with firmware flashing, servo calibration, temperature protection, speed tuning, WiFi configuration, WebSocket remote control, Bluetooth name management, and multilingual UI.

**No software installation required** — works with Chrome/Edge browser only.

### Firmware Versions

| Firmware | Source Directory | Description |
|----------|----------------|-------------|
| **With NTC (10K 3435)** | `SR6_OSR2_CraftyHandy_ESP32-NTC-10K/` | For NTC temp sensor PCB (10K 3435), auto-pause on overheat + LED alert |
| **With NTC (5K 3470)** | `SR6_OSR2_CraftyHandy_ESP32-NTC-5K/` | For NTC temp sensor PCB (5K 3470), auto-pause on overheat + LED alert |
| **Without NTC** | `SR6_OSR2_CraftyHandy_ESP32-NoNTC/` | For legacy users without NTC hardware, pure motion control |

Compile `.ino` with Arduino IDE or flash `.bin` from `build/` via the web tool.

### Web Tool (`index_wifi.html`)

#### Tabs

#### Tabs

| Tab | Function |
|-----|----------|
| **Flash** | Connect ESP32, select .bin, erase/write firmware, reboot |
| **Calibration** | SR6/OSR2 mode switch, servo zero adjustment, temp protection, speed tuning, TCode test sliders, funscript player |
| **Network** | Serial connection, WiFi config, Bluetooth name, WebSocket remote TCode control, funscript remote play |

#### Workflow

**First time (flashing)**
1. Open `index_wifi.html` in **Chrome/Edge**
2. "Flash" tab → Connect device → Select .bin → Write → Disconnect

**Calibration**
1. "Calibration" tab → Connect serial → Read/Write zero, temp, speed values
2. Use TCode sliders or funscript for testing

**WiFi & Remote Control**
1. "Network" tab → Connect serial
2. Enter SSID/password → Write → **Power cycle completely (unplug USB, then reconnect) — ESP32 only loads new WiFi config on cold boot**
3. Once connected, enter ESP32 IP → Connect → Use sliders/presets/funscript

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
| Valve | 26 |
| TwistFeedback | 25 |
| Vibe0 | 18 |
| Vibe1 | 19 |
| Lube (optional) | 23 |
| NTC (temp version) | 34 |
| Alarm LED (temp version) | 32 |

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

### Language Support

Language switcher in top-right corner supports **中文 / English / 日本語 / 한국어**.

### Bluetooth Name Persistence

Changing the Bluetooth name via `$B` saves to EEPROM automatically. The name persists across power cycles — the firmware loads it from EEPROM on startup and restarts Bluetooth with the saved name.

### WiFi Persistence

WiFi credentials saved via `$W` are stored in EEPROM. **A full power cycle (unplug USB / disconnect power) is required** — ESP32 only reads saved credentials at cold boot. After power cycling, it automatically connects using them.

### NTC Temperature Protection (NTC versions)

- Temperature checked every second
- Over threshold → auto-pause all motion + LED alarm blink
- Back below threshold → auto-resume motion
- **Sensor fault protection**: Invalid readings (outside -10°C ~ 120°C) are detected as sensor faults and skip protection logic to prevent false triggers

### WebSocket Remote Control

When ESP32 connects to WiFi, a WebSocket server starts on port 81. The web tool sends TCode commands via WebSocket for low-latency wireless control:

- Manual TCode command input
- 6-axis real-time sliders
- Preset action buttons (home, zero, scan)
- Funscript remote playback (origin
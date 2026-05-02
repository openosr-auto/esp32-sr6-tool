# CraftyHandy ESP32 SR6 Tool

> Website: [craftyhandy.com](https://craftyhandy.com/) | Email: openosr@gmail.com

---

## 🇨🇳 中文

### 简介
基于浏览器的 ESP32 SR6 调平工具，支持固件烧录、舵机校准、温度保护、速度微调、WiFi 控制等功能。

### 使用方式
1. 用 **Chrome/Edge** 浏览器打开 `esp32-tool.html`
2. 将 ESP32 通过 USB 连接到电脑
3. 点击「烧录」标签 → 连接设备 → 写入固件
4. 点击「校准」标签 → 连接串口 → 开始校准

### 功能说明

| 功能 | 说明 |
|------|------|
| **烧录固件** | 连接设备 → 选择 .bin → 写入/擦除/重启 |
| **舵机校准** | SR6(6轴) / OSR2(3轴) 模式切换，±10 微调舵机位置 |
| **温度保护** | 设置温度阈值，超过自动暂停运动 |
| **速度微调** | 6路舵机独立速度调节（0.1~5.0倍）|
| **测试动作** | 滑条控制 + .funscript 脚本播放 |
| **蓝牙名称** | 修改蓝牙名称 |
| **WiFi** | 配网后可通过浏览器发送 TCode 命令 |

### 固件命令
| 命令 | 功能 | 格式 |
|------|------|------|
| D3 / $Z | 调平值读写 | $Z1500,1500,1500,1500,1500,1500 |
| D4 / $T | 温度设置 | $T45.0,3.0 |
| D5 / $S | 速度系数 | $S1.0,1.0,1.0,1.0,1.0,1.0 |
| D6 / $B | 蓝牙名称 | $BMyDevice |
| D7 / $W | WiFi配置 | $WMySSID,MyPassword |
| L0-R2 | 6轴控制 | L05000 / R07500 |

---

## 🇬🇧 English

### Introduction
Browser-based ESP32 SR6 calibration tool with firmware flashing, servo calibration, temperature protection, speed tuning, and WiFi control.

### How to Use
1. Open `esp32-tool.html` with **Chrome/Edge**
2. Connect ESP32 via USB
3. Click "Flash" tab → Connect → Write firmware
4. Click "Calibration" tab → Connect serial → Start calibrating

### Features

| Feature | Description |
|---------|-------------|
| **Flash Firmware** | Connect → Select .bin → Write/Erase/Reboot |
| **Servo Calibration** | SR6(6-axis) / OSR2(3-axis) modes, ±10 fine tune |
| **Temp Protection** | Set temperature threshold, auto-pause on overheat |
| **Speed Tuning** | Independent speed for 6 servos (0.1~5.0x) |
| **Test Action** | Slider control + .funscript player |
| **Bluetooth Name** | Change Bluetooth device name |
| **WiFi** | Send TCode commands via browser after WiFi setup |

### Firmware Commands
| Command | Function | Format |
|---------|----------|--------|
| D3 / $Z | Read/Write calibration | $Z1500,1500,1500,1500,1500,1500 |
| D4 / $T | Temperature settings | $T45.0,3.0 |
| D5 / $S | Speed multiplier | $S1.0,1.0,1.0,1.0,1.0,1.0 |
| D6 / $B | Bluetooth name | $BMyDevice |
| D7 / $W | WiFi config | $WMySSID,MyPassword |
| L0-R2 | 6-axis control | L05000 / R07500 |

---

## 🇯🇵 日本語

### 概要
ブラウザベースのESP32 SR6調整ツール。ファームウェア書込み、サーボ調整、温度保護、速度調整、WiFi制御などをサポート。

### 使い方
1. **Chrome/Edge**で `esp32-tool.html` を開く
2. ESP32をUSB接続
3. 「書込み」タブ → 接続 → ファームウェア書込み
4. 「調整」タブ → シリアル接続 → 調整開始

### 機能一覧

| 機能 | 説明 |
|------|------|
| **ファームウェア書込み** | 接続 → .bin選択 → 書込/消去/再起動 |
| **サーボ調整** | SR6(6軸) / OSR2(3軸)モード切替、±10微調整 |
| **温度保護** | しきい値設定、超過時自動停止 |
| **速度調整** | 6軸独立速度調整（0.1~5.0倍）|
| **テスト動作** | スライダー制御 + .funscript再生 |
| **Bluetooth名** | Bluetooth名変更 |
| **WiFi** | WiFi設定後、ブラウザからTCode送信 |

---

## 🇰🇷 한국어

### 개요
브라우저 기반 ESP32 SR6 조정 도구. 펌웨어 쓰기, 서보 조정, 온도 보호, 속도 조정, WiFi 제어 등을 지원합니다.

### 사용 방법
1. **Chrome/Edge**로 `esp32-tool.html` 열기
2. ESP32를 USB로 연결
3. "쓰기" 탭 → 연결 → 펌웨어 쓰기
4. "조정" 탭 → 시리얼 연결 → 조정 시작

### 기능 설명

| 기능 | 설명 |
|------|------|
| **펌웨어 쓰기** | 연결 → .bin 선택 → 쓰기/지우기/재시작 |
| **서보 조정** | SR6(6축) / OSR2(3축) 모드, ±10 미세 조정 |
| **온도 보호** | 임계값 설정, 초과 시 자동 중지 |
| **속도 조정** | 6축 독립 속도 조정（0.1~5.0배）|
| **테스트 동작** | 슬라이더 제어 + .funscript 재생 |
| **블루투스 이름** | 블루투스 이름 변경 |
| **WiFi** | WiFi 설정 후 브라우저에서 TCode 전송 |

---

## Hardware Pin Mapping

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
| Vibe0 | 18 |
| Vibe1 | 19 |
| NTC Temp | 32 |
| Alarm LED | 33 |

---

*© CraftyHandy - openosr@gmail.com*

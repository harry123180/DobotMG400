# TCP-IP-Python-V4 完整技術文件 / Complete Technical Documentation

---

## 目錄 / Table of Contents

- [繁體中文版](#繁體中文版)
- [English Version](#english-version)

---

# 繁體中文版

## 一、專案概述

### 1.1 專案簡介

TCP-IP-Python-V4 是越疆機器人（Dobot）官方提供的 Python 二次開發 API 程式庫，用於透過 TCP/IP 協定控制越疆協作機器人（如 MG400、CR 系列等）。此 API 提供完整的機器人控制功能，包括：

- **運動控制**：關節運動（MovJ）、直線運動（MovL）、圓弧運動（Arc）等
- **狀態監控**：即時獲取機器人位置、速度、模式等資訊
- **IO 控制**：數位輸入/輸出、類比輸入/輸出
- **報警處理**：獲取並處理機器人報警資訊
- **座標系管理**：使用者座標系、工具座標系設定

### 1.2 系統架構

```
┌─────────────────────────────────────────────────────────────────┐
│                        使用者應用程式                             │
├─────────────────────────────────────────────────────────────────┤
│                      TCP-IP-Python-V4 API                        │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐              │
│  │ DobotApi    │  │ Dashboard   │  │ FeedBack    │              │
│  │ (基礎通訊)   │  │ (控制指令)   │  │ (狀態回饋)   │              │
│  └─────────────┘  └─────────────┘  └─────────────┘              │
├─────────────────────────────────────────────────────────────────┤
│                      TCP/IP 網路通訊                              │
│            Port 29999 (Dashboard)  /  Port 30004 (Feedback)      │
├─────────────────────────────────────────────────────────────────┤
│                        越疆機器人                                 │
└─────────────────────────────────────────────────────────────────┘
```

### 1.3 通訊埠說明

| 連接埠 | 用途 | 說明 |
|--------|------|------|
| 29999 | Dashboard | 發送控制指令、設定參數 |
| 30004 | Feedback | 接收機器人狀態回饋（1440 bytes/次） |
| 22000 | HTTP API | GetError 介面使用（報警資訊查詢） |

## 二、環境配置

### 2.1 系統需求

- **作業系統**：Windows / Linux / macOS
- **Python 版本**：3.6 或更高版本
- **網路環境**：與機器人在同一區域網路（192.168.X.X 網段）

### 2.2 安裝步驟

```bash
# 1. 安裝必要的 Python 套件
pip install numpy
pip install requests  # GetError 介面需要

# 2. 複製專案到本機
git clone https://github.com/Dobot-Arm/TCP-IP-CR-Python-V4.git

# 3. 配置網路
# - 將本機 IP 設定為 192.168.X.X 網段
# - 確保可以 ping 通機器人 IP
```

### 2.3 機器人設定

1. 開啟 DobotStudio Pro 軟體
2. 連線到機器人
3. 切換至 **TCP/IP 控制模式**
4. 記錄機器人的 IP 位址（預設通常為 192.168.5.1）

## 三、核心類別說明

### 3.1 DobotApi（基礎通訊類別）

最底層的通訊類別，負責 TCP Socket 連線管理。

```python
class DobotApi:
    def __init__(self, ip, port, *args)
    def send_data(self, string)      # 發送資料
    def wait_reply(self)              # 等待回覆
    def close(self)                   # 關閉連線
    def sendRecvMsg(self, string)     # 同步發送接收
```

### 3.2 DobotApiDashboard（控制指令類別）

繼承自 DobotApi，提供所有機器人控制指令。

#### 使能控制
```python
EnableRobot(load, centerX, centerY, centerZ, isCheck)  # 使能機器人
DisableRobot()                                          # 下使能機器人
```

#### 運動控制
```python
MovJ(x, y, z, rx, ry, rz, mode)    # 關節運動（點到點）
MovL(x, y, z, rx, ry, rz, mode)    # 直線運動
Arc(x1, y1, z1, rx1, ry1, rz1, x2, y2, z2, rx2, ry2, rz2)  # 圓弧運動
MoveJog(axis)                       # 點動控制
```

#### 參數設定
```python
SpeedFactor(ratio)        # 設定全域速度比例（1-100%）
User(index)               # 設定使用者座標系
Tool(index)               # 設定工具座標系
SetPayload(load, x, y, z) # 設定末端負載
```

#### IO 控制
```python
DO(index, status)         # 設定數位輸出
DI(index)                 # 讀取數位輸入
AO(index, value)          # 設定類比輸出
AI(index)                 # 讀取類比輸入
```

#### 報警處理
```python
GetErrorID()              # 獲取報警 ID（舊介面）
GetError(language)        # 獲取報警資訊（新介面，支援多語言）
ClearError()              # 清除報警
```

### 3.3 DobotApiFeedBack（狀態回饋類別）

即時接收機器人狀態資訊。

```python
class DobotApiFeedBack(DobotApi):
    def feedBackData(self)  # 獲取回饋資料
```

#### 回饋資料結構（MyType）

| 欄位名稱 | 資料型別 | 說明 |
|----------|----------|------|
| RobotMode | uint64 | 機器人模式 |
| QActual | float64[6] | 當前關節角度 |
| ToolVectorActual | float64[6] | 當前末端位姿 |
| DigitalInputs | uint64 | 數位輸入狀態 |
| DigitalOutputs | uint64 | 數位輸出狀態 |
| SpeedScaling | float64 | 當前速度比例 |
| CurrentCommandId | uint64 | 當前指令 ID |

#### 機器人模式對照表

| 模式值 | 狀態名稱 | 說明 |
|--------|----------|------|
| 1 | ROBOT_MODE_INIT | 初始化中 |
| 4 | ROBOT_MODE_DISABLED | 下使能狀態 |
| 5 | ROBOT_MODE_ENABLE | 使能但空閒 |
| 7 | ROBOT_MODE_RUNNING | 運動中 |
| 9 | ROBOT_MODE_ERROR | 報警狀態 |
| 10 | ROBOT_MODE_PAUSE | 暫停狀態 |
| 11 | ROBOT_MODE_JOG | 點動模式 |

## 四、使用範例

### 4.1 基本連線與使能

```python
from dobot_api import DobotApiDashboard, DobotApiFeedBack

# 連線到機器人
dashboard = DobotApiDashboard("192.168.5.1", 29999)
feedback = DobotApiFeedBack("192.168.5.1", 30004)

# 使能機器人
result = dashboard.EnableRobot()
print(f"使能結果: {result}")

# 設定速度為 50%
dashboard.SpeedFactor(50)

# 關閉連線
dashboard.close()
feedback.close()
```

### 4.2 運動控制範例

```python
from dobot_api import DobotApiDashboard
import time

dashboard = DobotApiDashboard("192.168.5.1", 29999)
dashboard.EnableRobot()

# 關節運動到指定位置（mode=1 表示使用關節角度）
dashboard.MovJ(0, -20, -80, 30, 90, 120, 1)

# 等待運動完成
time.sleep(3)

# 直線運動到笛卡爾座標（mode=0 表示使用笛卡爾座標）
dashboard.MovL(300, 0, 200, 180, 0, 0, 0)

dashboard.close()
```

### 4.3 即時狀態監控

```python
from dobot_api import DobotApiFeedBack
import numpy as np

feedback = DobotApiFeedBack("192.168.5.1", 30004)

while True:
    data = feedback.feedBackData()
    if data:
        print(f"機器人模式: {data['RobotMode'][0]}")
        print(f"關節角度: {data['QActual']}")
        print(f"末端位姿: {data['ToolVectorActual']}")
        print(f"速度比例: {data['SpeedScaling'][0]}%")
```

### 4.4 報警處理範例

```python
from dobot_api import DobotApiDashboard

dashboard = DobotApiDashboard("192.168.5.1", 29999)

# 獲取繁體中文報警資訊
error_info = dashboard.GetError("zh_hant")

if error_info and error_info.get("errMsg"):
    for error in error_info["errMsg"]:
        print(f"報警 ID: {error['id']}")
        print(f"描述: {error['description']}")
        print(f"解決方案: {error['solution']}")

    # 清除報警
    dashboard.ClearError()
else:
    print("無報警")
```

## 五、專案檔案結構

```
TCP-IP-Python-V4/
├── main.py                 # 主程式入口（命令列模式）
├── main_UI.py              # 主程式入口（圖形介面模式）
├── dobot_api.py            # 核心 API 函式庫
├── DobotDemo.py            # 示範類別
├── ui.py                   # Tkinter 圖形介面
├── get_error_example.py    # GetError 介面使用範例
├── GetError_README.md      # GetError 介面說明（繁體中文）
├── GetError_README_EN.md   # GetError 介面說明（英文）
├── README.md               # 專案說明文件
├── README-EN.md            # 專案說明文件（英文）
└── files/
    ├── alarmController.py  # 控制器報警碼對照表
    ├── alarmController.json
    ├── alarmServo.py       # 伺服報警碼對照表
    └── alarmServo.json
```

## 六、故障排除

### 6.1 常見錯誤

| 錯誤訊息 | 原因 | 解決方法 |
|----------|------|----------|
| Connection refused | 連接埠被佔用或機器人未開機 | 檢查連接埠、重啟機器人 |
| Control Mode Is Not Tcp | 機器人不在 TCP 模式 | 在 DobotStudio Pro 切換至 TCP 模式 |
| Robot is in error state | 機器人報警中 | 查看報警資訊並清除 |
| Timeout | 網路通訊超時 | 檢查網路連線、防火牆設定 |

### 6.2 除錯建議

1. 確保機器人和電腦在同一網段
2. 使用 `ping` 指令測試網路連通性
3. 檢查防火牆是否阻擋 29999、30004 連接埠
4. 確認沒有其他程式佔用相關連接埠

---

# English Version

## I. Project Overview

### 1.1 Introduction

TCP-IP-Python-V4 is the official Python secondary development API provided by Dobot for controlling Dobot collaborative robots (such as MG400, CR series, etc.) via TCP/IP protocol. This API provides complete robot control functionality including:

- **Motion Control**: Joint motion (MovJ), Linear motion (MovL), Arc motion (Arc), etc.
- **Status Monitoring**: Real-time acquisition of robot position, speed, mode, and other information
- **IO Control**: Digital input/output, Analog input/output
- **Alarm Handling**: Get and process robot alarm information
- **Coordinate System Management**: User coordinate system, Tool coordinate system settings

### 1.2 System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        User Application                          │
├─────────────────────────────────────────────────────────────────┤
│                      TCP-IP-Python-V4 API                        │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐              │
│  │ DobotApi    │  │ Dashboard   │  │ FeedBack    │              │
│  │ (Base Comm) │  │ (Control)   │  │ (Status)    │              │
│  └─────────────┘  └─────────────┘  └─────────────┘              │
├─────────────────────────────────────────────────────────────────┤
│                      TCP/IP Network                              │
│            Port 29999 (Dashboard)  /  Port 30004 (Feedback)      │
├─────────────────────────────────────────────────────────────────┤
│                        Dobot Robot                               │
└─────────────────────────────────────────────────────────────────┘
```

### 1.3 Port Description

| Port | Purpose | Description |
|------|---------|-------------|
| 29999 | Dashboard | Send control commands, set parameters |
| 30004 | Feedback | Receive robot status feedback (1440 bytes/time) |
| 22000 | HTTP API | GetError interface (alarm information query) |

## II. Environment Setup

### 2.1 System Requirements

- **Operating System**: Windows / Linux / macOS
- **Python Version**: 3.6 or higher
- **Network**: Same LAN as robot (192.168.X.X subnet)

### 2.2 Installation Steps

```bash
# 1. Install required Python packages
pip install numpy
pip install requests  # Required for GetError interface

# 2. Clone the project
git clone https://github.com/Dobot-Arm/TCP-IP-CR-Python-V4.git

# 3. Configure network
# - Set local IP to 192.168.X.X subnet
# - Ensure robot IP is reachable
```

### 2.3 Robot Configuration

1. Open DobotStudio Pro software
2. Connect to the robot
3. Switch to **TCP/IP control mode**
4. Note the robot's IP address (default is usually 192.168.5.1)

## III. Core Classes

### 3.1 DobotApi (Base Communication Class)

The lowest level communication class, responsible for TCP Socket connection management.

```python
class DobotApi:
    def __init__(self, ip, port, *args)
    def send_data(self, string)      # Send data
    def wait_reply(self)              # Wait for reply
    def close(self)                   # Close connection
    def sendRecvMsg(self, string)     # Synchronous send/receive
```

### 3.2 DobotApiDashboard (Control Command Class)

Inherits from DobotApi, provides all robot control commands.

#### Enable Control
```python
EnableRobot(load, centerX, centerY, centerZ, isCheck)  # Enable robot
DisableRobot()                                          # Disable robot
```

#### Motion Control
```python
MovJ(x, y, z, rx, ry, rz, mode)    # Joint motion (point-to-point)
MovL(x, y, z, rx, ry, rz, mode)    # Linear motion
Arc(x1, y1, z1, rx1, ry1, rz1, x2, y2, z2, rx2, ry2, rz2)  # Arc motion
MoveJog(axis)                       # Jog control
```

#### Parameter Settings
```python
SpeedFactor(ratio)        # Set global speed ratio (1-100%)
User(index)               # Set user coordinate system
Tool(index)               # Set tool coordinate system
SetPayload(load, x, y, z) # Set end effector payload
```

#### IO Control
```python
DO(index, status)         # Set digital output
DI(index)                 # Read digital input
AO(index, value)          # Set analog output
AI(index)                 # Read analog input
```

#### Alarm Handling
```python
GetErrorID()              # Get alarm ID (legacy interface)
GetError(language)        # Get alarm info (new interface, multi-language)
ClearError()              # Clear alarms
```

### 3.3 DobotApiFeedBack (Status Feedback Class)

Receives robot status information in real-time.

```python
class DobotApiFeedBack(DobotApi):
    def feedBackData(self)  # Get feedback data
```

#### Feedback Data Structure (MyType)

| Field Name | Data Type | Description |
|------------|-----------|-------------|
| RobotMode | uint64 | Robot mode |
| QActual | float64[6] | Current joint angles |
| ToolVectorActual | float64[6] | Current end effector pose |
| DigitalInputs | uint64 | Digital input status |
| DigitalOutputs | uint64 | Digital output status |
| SpeedScaling | float64 | Current speed ratio |
| CurrentCommandId | uint64 | Current command ID |

#### Robot Mode Reference

| Mode Value | Status Name | Description |
|------------|-------------|-------------|
| 1 | ROBOT_MODE_INIT | Initializing |
| 4 | ROBOT_MODE_DISABLED | Disabled state |
| 5 | ROBOT_MODE_ENABLE | Enabled but idle |
| 7 | ROBOT_MODE_RUNNING | In motion |
| 9 | ROBOT_MODE_ERROR | Error/Alarm state |
| 10 | ROBOT_MODE_PAUSE | Paused state |
| 11 | ROBOT_MODE_JOG | Jog mode |

## IV. Usage Examples

### 4.1 Basic Connection and Enable

```python
from dobot_api import DobotApiDashboard, DobotApiFeedBack

# Connect to robot
dashboard = DobotApiDashboard("192.168.5.1", 29999)
feedback = DobotApiFeedBack("192.168.5.1", 30004)

# Enable robot
result = dashboard.EnableRobot()
print(f"Enable result: {result}")

# Set speed to 50%
dashboard.SpeedFactor(50)

# Close connection
dashboard.close()
feedback.close()
```

### 4.2 Motion Control Example

```python
from dobot_api import DobotApiDashboard
import time

dashboard = DobotApiDashboard("192.168.5.1", 29999)
dashboard.EnableRobot()

# Joint motion to specified position (mode=1 means joint angles)
dashboard.MovJ(0, -20, -80, 30, 90, 120, 1)

# Wait for motion to complete
time.sleep(3)

# Linear motion to Cartesian coordinates (mode=0 means Cartesian)
dashboard.MovL(300, 0, 200, 180, 0, 0, 0)

dashboard.close()
```

### 4.3 Real-time Status Monitoring

```python
from dobot_api import DobotApiFeedBack
import numpy as np

feedback = DobotApiFeedBack("192.168.5.1", 30004)

while True:
    data = feedback.feedBackData()
    if data:
        print(f"Robot Mode: {data['RobotMode'][0]}")
        print(f"Joint Angles: {data['QActual']}")
        print(f"End Effector Pose: {data['ToolVectorActual']}")
        print(f"Speed Ratio: {data['SpeedScaling'][0]}%")
```

### 4.4 Alarm Handling Example

```python
from dobot_api import DobotApiDashboard

dashboard = DobotApiDashboard("192.168.5.1", 29999)

# Get alarm information in English
error_info = dashboard.GetError("en")

if error_info and error_info.get("errMsg"):
    for error in error_info["errMsg"]:
        print(f"Alarm ID: {error['id']}")
        print(f"Description: {error['description']}")
        print(f"Solution: {error['solution']}")

    # Clear alarms
    dashboard.ClearError()
else:
    print("No alarms")
```

## V. Project File Structure

```
TCP-IP-Python-V4/
├── main.py                 # Main entry (command line mode)
├── main_UI.py              # Main entry (GUI mode)
├── dobot_api.py            # Core API library
├── DobotDemo.py            # Demo class
├── ui.py                   # Tkinter GUI
├── get_error_example.py    # GetError interface example
├── GetError_README.md      # GetError documentation (Traditional Chinese)
├── GetError_README_EN.md   # GetError documentation (English)
├── README.md               # Project documentation
├── README-EN.md            # Project documentation (English)
└── files/
    ├── alarmController.py  # Controller alarm code lookup
    ├── alarmController.json
    ├── alarmServo.py       # Servo alarm code lookup
    └── alarmServo.json
```

## VI. Troubleshooting

### 6.1 Common Errors

| Error Message | Cause | Solution |
|---------------|-------|----------|
| Connection refused | Port occupied or robot not powered on | Check ports, restart robot |
| Control Mode Is Not Tcp | Robot not in TCP mode | Switch to TCP mode in DobotStudio Pro |
| Robot is in error state | Robot has alarm | Check alarm info and clear |
| Timeout | Network communication timeout | Check network connection, firewall settings |

### 6.2 Debugging Tips

1. Ensure robot and computer are on the same subnet
2. Use `ping` command to test network connectivity
3. Check if firewall blocks ports 29999, 30004
4. Confirm no other programs occupy related ports

---

**Document Version**: 1.0
**Last Updated**: 2025-01-05
**Author**: TCP-IP-Python-V4 Documentation Team

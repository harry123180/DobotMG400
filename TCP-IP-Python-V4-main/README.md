# TCP-IP-Python-V4 專案說明文件

## 專案概述

本專案是越疆機器人 TCP-IP-CR-Python-V4 二次開發 API 程式，用於透過 TCP/IP 協議控制越疆機器人。專案提供了完整的機器人控制介面，包括運動控制、狀態監控、報警處理等功能。

## 環境要求

### Python 版本

- Python 3.6 或更高版本

### 必需安裝的函式庫

```bash
# 基礎數值計算函式庫
pip install numpy

# JSON 資料處理（Python 內建，無需安裝）
# import json

# 網路通訊（Python 內建，無需安裝）
# import socket

# 多執行緒支援（Python 內建，無需安裝）
# import threading

# 時間處理（Python 內建，無需安裝）
# import time

# 正規表達式（Python 內建，無需安裝）
# import re

# GUI 介面函式庫（如果使用 ui.py）
pip install tkinter  # 通常 Python 自帶
```

### 網路配置要求

- 本機 IP 位址需設定為 192.168.X.X 網段
- 機器人需切換至 TCP/IP 模式
- 確保 29999 和 30004 連接埠未被佔用

## 主要程式檔案及功能

### 1. main.py

**功能**: 專案主入口檔案

- 展示基本的機器人連線和控制流程
- 包含完整的機器人操作範例
- 適合初學者了解專案結構

### 2. dobot_api.py

**功能**: 核心 API 介面檔案

- **DobotApi**: 基礎通訊類別，處理 TCP 連線
- **DobotApiDashboard**: 機器人控制介面類別
  - 機器人使能/下使能
  - 運動控制指令（MovJ, MovL, Arc 等）
  - 狀態查詢和設定
  - 報警資訊獲取（包含新增的 GetError 介面）
- **DobotApiFeedBack**: 狀態回饋類別
  - 即時獲取機器人狀態資訊
  - 監控機器人運行模式
  - 獲取當前指令 ID
- **MyType**: 資料型別定義
- **alarm_controller**: 控制器報警處理
- **alarm_servo**: 伺服報警處理

### 3. ui.py

**功能**: 圖形使用者介面程式

- 提供視覺化的機器人控制介面
- 整合了機器人連線、運動控制、狀態顯示等功能
- 支援即時顯示機器人狀態和報警資訊
- 優先使用 GetError 介面獲取報警資訊，失敗時回退到原有方式

### 4. 測試和範例檔案

#### get_error_example.py

**功能**: GetError 介面使用範例

- 提供 RobotErrorMonitor 類別，用於報警監控
- 展示如何獲取和處理多語言報警資訊
- 包含報警資訊儲存到檔案的功能
- 註解採用中英文對照

### 5. 文件檔案

#### GetError_README.md

**功能**: GetError 介面中文說明文件

- 詳細說明 GetError 介面的使用方法
- 包含介面參數、回傳值、範例程式碼等
- 提供故障排除和注意事項

#### GetError_README_EN.md

**功能**: GetError 介面英文說明文件

- GetError_README.md 的英文版本
- 便於國際使用者理解和使用

## 專案目錄結構

TCP-IP-Python-V4/
├── main.py                    # 主程式入口
├── dobot_api.py               # 核心 API 介面
├── ui.py                      # 圖形介面程式
├── PythonExample.py           # Python 範例
├── get_error_example.py       # GetError 使用範例
├── GetError_README.md         # GetError 中文文件
├── GetError_README_EN.md      # GetError 英文文件
├── README.md                  # 專案說明文件
└── files/                     # 其他支援檔案

## 快速開始

### 1. 環境準備

```bash
# 複製專案
git clone https://github.com/Dobot-Arm/TCP-IP-CR-Python-V4.git

# 安裝相依套件
pip install numpy
```

### 2. 網路配置

- 設定本機 IP 為 192.168.X.X 網段
- 確保機器人處於 TCP/IP 模式

### 3. 執行程式

# 執行主程式
python main.py

# 或執行圖形介面
python main_UI.py


## 常見問題解決

### 1. ModuleNotFoundError: No module named 'numpy'

**解決方法**: 安裝 numpy 函式庫

```bash
pip install numpy
```

### 2. Connection refused, IP:Port has been occupied

**解決方法**: 檢查 29999 連接埠是否被佔用，關閉佔用該連接埠的程式

### 3. Control Mode Is Not Tcp

**解決方法**: 在 DobotStudio Pro 中將機器人模式切換至 TCP/IP 模式

### 4. 機器人狀態異常

| 輸出訊息                             | 機器狀態     | 解決方法                 |
| ------------------------------------ | ------------ | ------------------------ |
| Command execution failed             | 指令執行失敗 | 檢查指令參數和機器人狀態 |
| The robot is in an error state       | 機器錯誤狀態 | 清除報警後重試           |
| The robot is in emergency stop state | 急停狀態     | 釋放急停按鈕             |
| The robot is in power down state     | 下電狀態     | 給機器人上電             |

## 注意事項

1. **安全第一**: 執行範例前請確保機器人處於安全位置，防止發生碰撞
2. **網路配置**: 確保網路配置正確，IP 位址在同一網段
3. **連接埠佔用**: 確保 29999 和 30004 連接埠未被其他程式佔用
4. **機器人模式**: 確保機器人處於 TCP/IP 控制模式
5. **權限問題**: 某些操作可能需要管理員權限

## 技術支援

如遇到問題，請參考：專案 README.md 文件

- GetError 相關文件
- 範例程式碼和測試程式
- 越疆官方技術支援

---

**版本**: V4
**更新日期**: 2025-9-5
**維護**: dobot_futingxing

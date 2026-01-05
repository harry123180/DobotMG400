# Dobot MG400

## 規格 Specifications

### 繁體中文

| 項目 | 規格 |
|------|------|
| 軸數 | 4 |
| 有效載荷 | 500g（最大 750g） |
| 工作半徑 | 440mm |
| 重複定位精度 | ±0.05mm |
| **運動範圍** | |
| J1 | ±160° |
| J2 | -25° ~ 85° |
| J3 | -25° ~ 105° |
| J4 | ±360° |
| **最大關節速度** | |
| J1 | 300°/s |
| J2 | 300°/s |
| J3 | 300°/s |
| J4 | 300°/s |
| **電源與功耗** | |
| 輸入電源 | 100~240V AC, 50/60Hz |
| 額定電壓 | 48V |
| 功耗 | 150W |
| **通訊與安裝** | |
| 通訊介面 | TCP/IP, Modbus TCP |
| 安裝方式 | 桌面安裝 |
| 重量 | 8kg |
| 底座尺寸 | 190mm × 190mm |
| 工作環境溫度 | 0°C ~ 40°C |

---

### English

| Item | Specification |
|------|---------------|
| Number of Axes | 4 |
| Payload | 500g (Max 750g) |
| Working Radius | 440mm |
| Repeatability | ±0.05mm |
| **Motion Range** | |
| J1 | ±160° |
| J2 | -25° ~ 85° |
| J3 | -25° ~ 105° |
| J4 | ±360° |
| **Max Joint Speed** | |
| J1 | 300°/s |
| J2 | 300°/s |
| J3 | 300°/s |
| J4 | 300°/s |
| **Power** | |
| Input Power | 100~240V AC, 50/60Hz |
| Rated Voltage | 48V |
| Power Consumption | 150W |
| **Communication & Installation** | |
| Communication | TCP/IP, Modbus TCP |
| Installation | Desktop |
| Weight | 8kg |
| Base Size | 190mm × 190mm |
| Operating Temperature | 0°C ~ 40°C |

---

## 專案結構 Project Structure

```
DobotMG400/
├── README.md                                           # 本文件 / This file
├── MG400 Model (Creo4.0)-20210406.zip                 # Creo 4.0 3D 模型
├── MG400 Model (Solidworks2014)-20210406.zip          # SolidWorks 2014 3D 模型
├── MG400_End_Flange_3D.stp                            # 末端法蘭 3D 模型 (STEP 格式)
├── TCP_IP Remote Control Interface Guide (4axis)_20240419_en.pdf
│                                                       # TCP/IP 遠端控制介面指南 (英文)
└── TCP-IP-Protocol-4AXis-master/
    ├── TCP_IP Remote Control Interface Guide (4axis)_20240419_en.pdf
    │                                                   # TCP/IP 遠端控制介面指南 (英文)
    └── TCP_IP远程控制接口文档（4轴）_20240419_cn.pdf
                                                        # TCP/IP 遠端控制介面指南 (簡體中文)
```

### 檔案說明 File Description

| 檔案 / File | 說明 / Description |
|-------------|---------------------|
| `MG400 Model (Creo4.0)-20210406.zip` | MG400 機械手臂 3D 模型，適用於 PTC Creo 4.0 及以上版本 |
| `MG400 Model (Solidworks2014)-20210406.zip` | MG400 機械手臂 3D 模型，適用於 SolidWorks 2014 及以上版本 |
| `MG400_End_Flange_3D.stp` | 末端法蘭 STEP 格式 3D 模型，可用於設計末端執行器夾具 |
| `TCP_IP Remote Control Interface Guide` | TCP/IP 通訊協議文件，包含指令格式與 API 說明 |

---

## 相關連結 Related Links

- [Dobot 官方網站](https://www.dobot-robots.com/)
- [MG400 產品頁面](https://www.dobot-robots.com/products/desktop-four-axis/mg400.html)

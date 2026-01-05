# GetError 介面使用說明

## 概述

`GetError` 介面是在 `dobot_api.py` 中新增的功能，用於透過 HTTP 請求獲取機器人的報警資訊。該介面支援多種語言，並回傳 JSON 格式的報警資料。

## 功能特性

- ✅ 支援 10 種語言的報警資訊顯示
- ✅ 透過 HTTP GET 請求獲取報警資料
- ✅ 自動設定語言偏好
- ✅ 回傳結構化的 JSON 資料
- ✅ 完善的錯誤處理機制

## 支援的語言

| 語言代碼    | 語言名稱              |
| ----------- | --------------------- |
| `zh_cn`   | 簡體中文              |
| `zh_hant` | 繁體中文              |
| `en`      | English (英語)        |
| `ja`      | 日本語 (日語)         |
| `de`      | Deutsch (德語)        |
| `vi`      | Tiếng Việt (越南語) |
| `es`      | Español (西班牙語)   |
| `fr`      | Français (法語)      |
| `ko`      | 한국어 (韓語)         |
| `ru`      | Русский (俄語) |

## 介面說明

### 方法簽章

```python
def GetError(self, language="zh_cn"):
    """
    獲取機器人報警資訊

    Args:
        language (str): 語言設定，預設為 "zh_cn"

    Returns:
        dict: 報警資訊字典，格式如下：
        {
            "errMsg": [
                {
                    "id": xxx,
                    "level": xxx,
                    "description": "xxx",
                    "solution": "xxx",
                    "mode": "xxx",
                    "date": "xxxx",
                    "time": "xxxx"
                }
            ]
        }
    """
```

### 使用的 HTTP 介面

1. **設定語言介面**

   - **URL**: `http://ip:22000/interface/language`
   - **方法**: POST
   - **資料格式**: `{"type": "語言代碼"}`
2. **獲取報警資訊介面**

   - **URL**: `http://ip:22000/protocol/getAlarm`
   - **方法**: GET
   - **回傳格式**: JSON

## 基本使用範例

### 1. 簡單使用

```python
from dobot_api import DobotApiDashboard

# 建立連線
dashboard = DobotApiDashboard("192.168.5.1", 29999)

# 獲取中文報警資訊
error_info = dashboard.GetError("zh_cn")

# 檢查是否有報警
if error_info and "errMsg" in error_info:
    errors = error_info["errMsg"]
    if errors:
        print(f"發現 {len(errors)} 個報警")
        for error in errors:
            print(f"報警 ID: {error['id']}")
            print(f"描述: {error['description']}")
            print(f"解決方案: {error['solution']}")
    else:
        print("無報警資訊")

# 關閉連線
dashboard.close()
```

### 2. 多語言支援

```python
# 獲取英文報警資訊
error_info_en = dashboard.GetError("en")

# 獲取日文報警資訊
error_info_ja = dashboard.GetError("ja")

# 獲取德文報警資訊
error_info_de = dashboard.GetError("de")
```

### 3. 錯誤處理

```python
try:
    error_info = dashboard.GetError("zh_cn")
    if error_info is None:
        print("獲取報警資訊失敗")
    elif "errMsg" not in error_info:
        print("回傳資料格式異常")
    else:
        # 處理正常資料
        pass
except Exception as e:
    print(f"發生異常: {e}")
```

## 完整範例程式

專案中提供了兩個範例程式：

1. **`test_get_error.py`** - 基本功能測試
2. **`get_error_example.py`** - 完整的使用範例，包含監控類別

### 執行測試程式

```bash
# 基本測試
python test_get_error.py

# 完整範例
python get_error_example.py
```

## 回傳資料格式

### 成功回應

```json
{
    "errMsg": [
        {
            "id": 1537,
            "level": 1,
            "description": "急停按鈕被拍下",
            "solution": "恢復急停按鈕並清錯。若仍然告警，則確認急停按鈕是否損壞，可更換急停按鈕",
            "mode": "安全控制器錯誤",
            "date": "2025-01-09",
            "time": "10:30:15"
        }
    ]
}
```

### 無報警時

```json
{
    "errMsg": []
}
```

### 錯誤情況

- 網路連線失敗：回傳 `None`
- HTTP 請求失敗：回傳 `None`
- JSON 解析失敗：回傳 `None`

## 注意事項

1. **網路連線**：確保機器人網路連線正常，連接埠 22000 可存取
2. **IP 位址**：使用建立 TCP 連線時的相同 IP 位址
3. **語言設定**：每次呼叫都會先設定語言，然後獲取報警資訊
4. **錯誤處理**：建議在實際使用中添加適當的錯誤處理邏輯
5. **連線管理**：使用完畢後記得關閉連線

## 故障排除

### 常見問題

1. **連線逾時**

   - 檢查機器人 IP 位址是否正確
   - 確認網路連線是否正常
   - 驗證連接埠 22000 是否開放
2. **回傳 None**

   - 檢查 HTTP 請求是否成功
   - 確認機器人韌體版本是否支援該介面
   - 查看網路防火牆設定
3. **語言設定無效**

   - 確認語言代碼拼寫正確
   - 檢查機器人是否支援該語言
   - 嘗試使用預設語言 "zh_cn"

### 除錯建議

1. 先使用 `test_get_error.py` 進行基本功能測試
2. 檢查機器人 Web 介面是否可以正常存取
3. 使用瀏覽器直接存取 HTTP 介面進行驗證
4. 查看主控台輸出的錯誤訊息

## 更新日誌

- **v1.0** (2025-09-05)
  - 初始版本發布
  - 支援 10 種語言
  - 完整的錯誤處理機制
  - 提供範例程式和文件

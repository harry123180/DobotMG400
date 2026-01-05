#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GetError Interface Usage Example / GetError 介面使用範例
Demonstrates how to use the GetError interface to get robot alarm information in real projects
展示如何在實際專案中使用 GetError 介面獲取機器人報警資訊
"""

from dobot_api import DobotApiDashboard
import time
import json

class RobotErrorMonitor:
    """
    Robot Error Monitor Class / 機器人報警監控類別
    A class for monitoring robot alarm information
    用於監控機器人報警資訊的類別
    """
    
    def __init__(self, robot_ip="192.168.200.1", dashboard_port=29999):
        self.robot_ip = robot_ip
        self.dashboard_port = dashboard_port
        self.dashboard = None
        
    def connect(self):
        """Connect to robot / 連線到機器人"""
        try:
            self.dashboard = DobotApiDashboard(self.robot_ip, self.dashboard_port)
            print(f"Successfully connected to robot / 成功連線到機器人: {self.robot_ip}:{self.dashboard_port}")
            return True
        except Exception as e:
            print(f"Failed to connect to robot / 連線機器人失敗: {e}")
            return False

    def disconnect(self):
        """Disconnect from robot / 斷開連線"""
        if self.dashboard:
            self.dashboard.close()
            print("Disconnected from robot / 已斷開機器人連線")
    
    def get_error_info(self, language="zh_cn"):
        """
        Get error information / 獲取報警資訊

        Args:
            language (str): Language setting, supports / 語言設定，支援:
                           "zh_cn" - Simplified Chinese / 簡體中文
                           "zh_hant" - Traditional Chinese / 繁體中文
                           "en" - English / 英語
                           "ja" - Japanese / 日語
                           "de" - German / 德語
                           "vi" - Vietnamese / 越南語
                           "es" - Spanish / 西班牙語
                           "fr" - French / 法語
                           "ko" - Korean / 韓語
                           "ru" - Russian / 俄語

        Returns:
            dict: Error information dictionary / 報警資訊字典
        """
        if not self.dashboard:
            print("Not connected to robot / 未連線到機器人")
            return None

        return self.dashboard.GetError(language)
    
    def check_errors(self, language="zh_cn"):
        """
        Check and display current error information / 檢查並顯示當前報警資訊

        Args:
            language (str): Display language / 顯示語言

        Returns:
            bool: True means there are errors, False means no errors / True 表示有報警，False 表示無報警
        """
        error_info = self.get_error_info(language)

        if not error_info or "errMsg" not in error_info:
            print("Failed to get error information / 獲取報警資訊失敗")
            return False

        errors = error_info["errMsg"]

        if not errors:
            print("✅ Robot status normal, no error information / 機器人狀態正常，無報警資訊")
            return False

        print(f"⚠️  Found {len(errors)} error(s) / 發現 {len(errors)} 個報警:")
        print("=" * 50)

        for i, error in enumerate(errors, 1):
            print(f"Error / 報警 {i}:")
            print(f"  🆔 ID: {error.get('id', 'N/A')}")
            print(f"  📊 Level / 級別: {error.get('level', 'N/A')}")
            print(f"  📝 Description / 描述: {error.get('description', 'N/A')}")
            print(f"  🔧 Solution / 解決方案: {error.get('solution', 'N/A')}")
            print(f"  🏷️  Mode / 模式: {error.get('mode', 'N/A')}")
            print(f"  📅 Date / 日期: {error.get('date', 'N/A')}")
            print(f"  🕐 Time / 時間: {error.get('time', 'N/A')}")
            print("-" * 30)

        return True
    
    def monitor_errors(self, interval=5, language="zh_cn"):
        """
        Continuously monitor error information / 持續監控報警資訊

        Args:
            interval (int): Check interval (seconds) / 檢查間隔（秒）
            language (str): Display language / 顯示語言
        """
        print(f"Start monitoring robot error information (check every {interval} seconds) / 開始監控機器人報警資訊（每 {interval} 秒檢查一次）")
        print("Press Ctrl+C to stop monitoring / 按 Ctrl+C 停止監控")

        try:
            while True:
                print(f"\n[{time.strftime('%Y-%m-%d %H:%M:%S')}] Checking error information / 檢查報警資訊...")
                has_errors = self.check_errors(language)

                if has_errors:
                    print("\n⚠️  Recommend handling error information immediately / 建議立即處理報警資訊！")

                time.sleep(interval)

        except KeyboardInterrupt:
            print("\nMonitoring stopped / 監控已停止")
    
    def save_error_log(self, filename=None, language="zh_cn"):
        """
        Save error information to file / 儲存報警資訊到檔案

        Args:
            filename (str): Save filename, default is current timestamp / 儲存檔案名稱，預設為當前時間戳記
            language (str): Language setting / 語言設定
        """
        if filename is None:
            filename = f"robot_errors_{time.strftime('%Y%m%d_%H%M%S')}.json"

        error_info = self.get_error_info(language)

        if error_info:
            try:
                with open(filename, 'w', encoding='utf-8') as f:
                    json.dump(error_info, f, ensure_ascii=False, indent=2)
                print(f"Error information saved to / 報警資訊已儲存到: {filename}")
            except Exception as e:
                print(f"Failed to save file / 儲存檔案失敗: {e}")
        else:
            print("Unable to get error information / 無法獲取報警資訊")

def main():
    """Main function - Demonstrate various usage methods / 主函式 - 展示各種使用方式"""

    # Create monitor instance / 建立監控器實例
    monitor = RobotErrorMonitor()

    # Connect to robot / 連線機器人
    if not monitor.connect():
        return

    try:
        print("\n=== GetError Interface Usage Example / GetError 介面使用範例 ===")

        # 1. Basic usage - Check current errors / 基本使用 - 檢查當前報警
        print("\n1. Check current error information / 檢查當前報警資訊:")
        monitor.check_errors("zh_cn")

        # 2. Multi-language support / 多語言支援
        print("\n2. Multi-language support demonstration / 多語言支援展示:")
        languages = {
            "zh_cn": "簡體中文 / Simplified Chinese",
            "en": "English / 英語",
            "ja": "日本語 / Japanese"
        }

        for lang_code, lang_name in languages.items():
            print(f"\n--- {lang_name} ({lang_code}) ---")
            monitor.check_errors(lang_code)

        # 3. Save error log / 儲存報警日誌
        print("\n3. Save error log / 儲存報警日誌:")
        monitor.save_error_log()

        # 4. Get raw data / 獲取原始資料
        print("\n4. Get raw JSON data / 獲取原始 JSON 資料:")
        raw_data = monitor.get_error_info("zh_cn")
        if raw_data:
            print(json.dumps(raw_data, ensure_ascii=False, indent=2))

        # 5. Optional: Start continuous monitoring (uncomment to enable) / 可選：啟動持續監控（取消註解以啟用）
        # print("\n5. Start continuous monitoring / 啟動持續監控:")
        # monitor.monitor_errors(interval=10, language="zh_cn")

    finally:
        # Disconnect / 斷開連線
        monitor.disconnect()

if __name__ == "__main__":
    main()
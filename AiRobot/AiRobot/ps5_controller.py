#!/usr/bin/env python3
"""
PS5コントローラー入力をTeensy 4.0にシリアル通信で送るスクリプト
PSボタンでシリアル通信のオンオフ切り替え可能
"""

import pygame
import serial
import serial.tools.list_ports
import time
import sys
import threading
from typing import Optional

class PS5Controller:
    def __init__(self, serial_port: str = "AUTO", baud_rate: int = 115200):
        """
        PS5コントローラーの初期化
        Args:
            serial_port: Teensyが接続されているCOMポート
            baud_rate: ボーレート（デフォルト115200）
        """
        # Pygame初期化
        pygame.init()
        pygame.joystick.init()
        
        # コントローラー接続確認
        if pygame.joystick.get_count() == 0:
            print("PS5コントローラーが見つかりません")
            sys.exit(1)
        
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        print(f"コントローラー接続: {self.joystick.get_name()}")
        
        # シリアル通信設定
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.serial_conn: Optional[serial.Serial] = None
        self.serial_enabled = False
        self.ps_button_pressed = False
        
        # コントローラー値
        self.L_x = 0
        self.L_y = 0
        self.R_x = 0
        self.R_y = 0
        
        # デッドゾーン設定
        self.deadzone = 0.05
        
        # 送信頻度制御
        self.last_send_time = 0
        self.send_interval = 0.01  # 100Hz (10ms間隔)

        self.find_and_connect_serial()
    
    def find_and_connect_serial(self):
        """利用可能なCOMポートを検出してTeensyに接続を試行"""
        # まず指定されたポートを試行（AUTOでない場合）
        if self.serial_port != "AUTO" and self.connect_serial_port(self.serial_port):
            return True
        
        # 指定ポートが失敗した場合、自動検出
        if self.serial_port == "AUTO":
            print("自動検出を開始...")
        else:
            print("指定ポートに接続できませんでした。自動検出を開始...")
        
        ports = serial.tools.list_ports.comports()
        teensy_ports = []
        
        for port in ports:
            # Teensyの特徴的な文字列を検索
            desc_lower = port.description.lower()
            hwid_lower = port.hwid.lower()
            
            # TeensyのVID:PID (16C0:0483) またはUSB シリアル デバイスをチェック
            is_teensy = (
                'teensy' in desc_lower or
                ('usb' in desc_lower and 'serial' in desc_lower and 'bluetooth' not in desc_lower) or
                '16c0:0483' in hwid_lower
            )
            
            if is_teensy:
                teensy_ports.append(port.device)
                print(f"Teensy候補発見: {port.device} - {port.description}")
        
        # Teensyらしきポートを試行
        for port in teensy_ports:
            print(f"接続試行: {port}")
            if self.connect_serial_port(port):
                self.serial_port = port
                return True
        
        # 全ポートを試行（最後の手段）
        print("全COMポートを試行します...")
        for port in ports:
            if self.connect_serial_port(port.device):
                self.serial_port = port.device
                return True
        
        print("利用可能なCOMポートが見つかりませんでした")
        return False
    
    def connect_serial_port(self, port: str):
        """指定されたポートに接続を試行"""
        try:
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()
                
            self.serial_conn = serial.Serial(
                port, 
                self.baud_rate, 
                timeout=0.01,
                write_timeout=0.01
            )
            print(f"シリアル接続成功: {port}")
            return True
        except serial.SerialException:
            return False
    
    def apply_deadzone(self, value: float) -> float:
        """デッドゾーンを適用"""
        if abs(value) < self.deadzone:
            return 0.0
        return value
    
    def map_axis_value(self, value: float, scale: int = 100) -> int:
        """軸の値を-scale～+scaleの範囲にマッピング"""
        value = self.apply_deadzone(value)
        return int(value * scale)
    
    def read_controller(self):
        """コントローラー入力を読み取り"""
        pygame.event.pump()
        
        # アナログスティック値取得（左スティック: 軸0,1 / 右スティック: 軸2,3）
        left_x_raw = self.joystick.get_axis(0)   # 左スティック X軸
        left_y_raw = -self.joystick.get_axis(1)  # 左スティック Y軸（反転）
        right_x_raw = self.joystick.get_axis(2)  # 右スティック X軸
        right_y_raw = -self.joystick.get_axis(3) # 右スティック Y軸（反転）
        
        # マッピング（-100～+100の範囲）
        self.L_x = self.map_axis_value(left_x_raw)
        self.L_y = self.map_axis_value(left_y_raw)
        self.R_x = self.map_axis_value(right_x_raw)
        self.R_y = self.map_axis_value(right_y_raw)
        
        # PSボタンまたは代替ボタンの状態確認
        # PSボタン候補: 10, 11, 12, 13, 16, 17
        # 代替ボタン: 0(X), 1(○), 2(□), 3(△)
        toggle_button_current = False
        button_name = ""
        
        # まずPSボタンをチェック（ボタン15）
        if self.joystick.get_button(15):
            toggle_button_current = True
            button_name = "PSボタン"
        
        # PSボタンが見つからない場合は△ボタン（ボタン3）を使用
        if not toggle_button_current and self.joystick.get_button(3):
            toggle_button_current = True
            button_name = "△ボタン"
        
        # トグルボタンが押された瞬間を検出（トグル動作）
        if toggle_button_current and not self.ps_button_pressed:
            self.serial_enabled = not self.serial_enabled
            status = "ON" if self.serial_enabled else "OFF"
            print(f"\n{button_name}でシリアル通信: {status}")
        
        self.ps_button_pressed = toggle_button_current
    
    def send_to_teensy(self):
        """Teensyにデータを送信（頻度制限付き）"""
        if not self.serial_enabled or not self.serial_conn:
            return
        
        # 送信頻度制限
        current_time = time.time()
        if current_time - self.last_send_time < self.send_interval:
            return
        
        try:
            # Teensyが期待する形式: "L_x,L_y,R_x,R_y\n"
            message = f"{self.L_x},{self.L_y},{self.R_x},{self.R_y}\n"
            # シリアルポートが開いているか確認
            if not self.serial_conn.is_open:
                print("シリアルポートが閉じられています")
                self.serial_enabled = False
                return
                
            # 非ブロッキング書き込み
            self.serial_conn.write(message.encode())
            self.serial_conn.flush()  # バッファを強制的にフラッシュ
            self.last_send_time = current_time
            
        except (serial.SerialException, serial.SerialTimeoutException) as e:
            print(f"\nシリアル送信エラー: {e}")
            self.serial_enabled = False
            self.serial_conn = None
            print("シリアル通信を無効にしました")
        except Exception as e:
            print(f"\n予期しないエラー: {e}")
            self.serial_enabled = False
    
    def display_status(self):
        """現在の状態を表示"""
        status = "ON " if self.serial_enabled else "OFF"
        print(f"\rSerial: {status} | L_x:{self.L_x:4d} L_y:{self.L_y:4d} R_x:{self.R_x:4d} R_y:{self.R_y:4d}", end="")
    
    def run(self):
        """メインループ"""
        print("PS5コントローラー開始")
        print("PSボタンまたは△ボタンでシリアル通信のオンオフを切り替えます")
        print("Ctrl+Cで終了")
        
        try:
            while True:
                self.read_controller()
                self.send_to_teensy()
                self.display_status()
                time.sleep(0.02)  # 50Hz更新（コントローラー読み取り）
                
        except KeyboardInterrupt:
            print("\n終了します")
        finally:
            if self.serial_conn:
                self.serial_conn.close()
            pygame.quit()

def main():
    """メイン関数"""
    # COMポートを必要に応じて変更してください
    controller = PS5Controller(serial_port="AUTO", baud_rate=115200)
    controller.run()

if __name__ == "__main__":
    main()
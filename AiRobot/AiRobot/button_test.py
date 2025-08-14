#!/usr/bin/env python3
"""
PS5コントローラーのボタンマッピングをテストするスクリプト
全てのボタンと軸の状態を表示してPSボタンの番号を特定する
"""

import pygame
import time
import sys

def test_controller():
    """コントローラーの全ボタンと軸をテスト"""
    # Pygame初期化
    pygame.init()
    pygame.joystick.init()
    
    # コントローラー接続確認
    if pygame.joystick.get_count() == 0:
        print("PS5コントローラーが見つかりません")
        sys.exit(1)
    
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    
    print(f"コントローラー名: {joystick.get_name()}")
    print(f"ボタン数: {joystick.get_numbuttons()}")
    print(f"軸数: {joystick.get_numaxes()}")
    print(f"ハット数: {joystick.get_numhats()}")
    print("\n各ボタンを押してPSボタンの番号を確認してください")
    print("Ctrl+Cで終了")
    print("-" * 60)
    
    try:
        while True:
            pygame.event.pump()
            
            # 軸の状態表示
            axes_info = []
            for i in range(joystick.get_numaxes()):
                value = joystick.get_axis(i)
                if abs(value) > 0.1:  # デッドゾーン
                    axes_info.append(f"軸{i}:{value:.2f}")
            
            # ボタンの状態表示
            pressed_buttons = []
            for i in range(joystick.get_numbuttons()):
                if joystick.get_button(i):
                    pressed_buttons.append(f"ボタン{i}")
            
            # ハット（十字キー）の状態表示
            hat_info = []
            for i in range(joystick.get_numhats()):
                hat_value = joystick.get_hat(i)
                if hat_value != (0, 0):
                    hat_info.append(f"ハット{i}:{hat_value}")
            
            # 状態が変化した時のみ表示
            if axes_info or pressed_buttons or hat_info:
                status_line = ""
                if axes_info:
                    status_line += " | ".join(axes_info) + " | "
                if pressed_buttons:
                    status_line += " | ".join(pressed_buttons) + " | "
                if hat_info:
                    status_line += " | ".join(hat_info)
                
                print(f"\r{status_line:<80}", end="")
                
                # PSボタンの可能性があるボタンを特別表示
                for i in range(joystick.get_numbuttons()):
                    if joystick.get_button(i):
                        # よく使われるPSボタン番号をチェック
                        if i in [10, 11, 12, 13, 16, 17]:
                            print(f"\n>>> PSボタンの可能性: ボタン{i} <<<")
            
            time.sleep(0.05)  # 20Hz更新
            
    except KeyboardInterrupt:
        print("\n終了します")
    finally:
        pygame.quit()

if __name__ == "__main__":
    test_controller()